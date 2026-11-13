"""Request lifecycle and authoritative GitHub-job-based reporting."""
from urllib.error import HTTPError

TERMINAL = {'success', 'failure', 'error', 'superseded'}


def run_url(repository, run_id):
    return f'https://github.com/{repository}/actions/runs/{run_id}'


def outcome(record, jobs):
    if record['state'] == 'preparation_failed':
        return 'error', 'Input preparation failed; see central run'
    expected = 'source-test:' + record['request']['id']
    matches = [job for job in jobs if job['name'].split(' / ')[-1] == expected]
    if len(matches) != 1:
        return 'error', 'Source worker did not complete; see central run'
    conclusion = matches[0]['conclusion']
    if conclusion == 'success':
        return 'success', 'Isolated source build and tests passed'
    if conclusion == 'failure':
        return 'failure', 'Isolated source build or tests failed'
    return 'error', 'Source worker cancelled, skipped, or timed out'


def finish(state, key, record, etag, jobs, github):
    if record['state'] in TERMINAL:
        return
    result, description = outcome(record, jobs)
    github.status(record['request'], result, description,
                  run_url(record['execution_repository'], record['run_id']))
    record['state'] = result
    state.put(key, record, etag)


def reconcile(state, github, actions):
    runs = {}
    for key, record, etag in state.records():
        if record['state'] in TERMINAL:
            continue
        owner = (record['execution_repository'], record['run_id'])
        if owner not in runs:
            run = actions.request('GET', f'/repos/{owner[0]}/actions/runs/{owner[1]}')
            runs[owner] = (list(actions.pages(f'/repos/{owner[0]}/actions/runs/{owner[1]}/jobs', 'jobs'))
                           if run['status'] == 'completed' else None)
        if runs[owner] is not None:
            finish(state, key, record, etag, runs[owner], github)


def enqueue(requests, state, github, actions, execution_repository, run_id, attempt, limit=4, retry_failed=False):
    existing = list(state.records())
    newest = {request['family']: request['id'] for request in requests}
    cancel = set()
    for key, record, etag in existing:
        request = record['request']
        if (record['state'] not in TERMINAL and request['family'] in newest
                and newest[request['family']] != request['id']):
            github.status(request, 'error', 'Superseded by a newer source/environment revision',
                          run_url(record['execution_repository'], record['run_id']))
            record['state'] = 'superseded'
            state.put(key, record, etag)
            cancel.add((record['execution_repository'], record['run_id']))
    for repository, old_run in cancel:
        if str(old_run) != str(run_id) and not any(
            r['state'] not in TERMINAL and r['execution_repository'] == repository and r['run_id'] == old_run
            for _, r, _ in existing
        ):
            try:
                actions.request('POST', f'/repos/{repository}/actions/runs/{old_run}/cancel')
            except HTTPError as error:
                if error.code != 409:  # It may have finished between discovery and cancellation.
                    raise
    claimed = []
    for request in requests:
        if len(claimed) >= limit:
            break
        key = 'requests/' + request['id'] + '.json'
        previous, etag = state.get(key)
        if previous and not (retry_failed and previous['state'] in ('failure', 'error', 'superseded')):
            continue
        record = dict(request=request, state='queued', execution_repository=execution_repository,
                      run_id=run_id, attempt=attempt)
        state.put(key, record, etag)
        github.status(request, 'pending', 'Queued for isolated source CI', run_url(execution_repository, run_id))
        claimed.append(request['id'])
    state.put(f'runs/{run_id}-{attempt}.json', {'requests': claimed})
    return claimed
