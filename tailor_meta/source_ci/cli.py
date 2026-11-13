"""Trusted control-plane entry points. Never invoke these on test workers."""
import argparse
import json
import os
from pathlib import Path
import re
import tempfile

import boto3

from .controller import enqueue, reconcile, finish
from .github import GitHub
from .inputs import extract_source
from .poll import discover
from .prepare import prepare
from .state import State


def owned(state, request_id):
    if not request_id or not re.fullmatch(r'[a-f0-9]{64}', request_id):
        raise ValueError('Invalid source-CI request identity')
    key = 'requests/' + request_id + '.json'
    record, etag = state.get(key)
    if (not record or record['execution_repository'] != os.environ['GITHUB_REPOSITORY']
            or str(record['run_id']) != os.environ['GITHUB_RUN_ID']
            or str(record['attempt']) != os.environ['GITHUB_RUN_ATTEMPT']):
        raise ValueError('This Actions run does not own the source-CI request')
    return key, record, etag


def artifact_policy(actions, allow_public):
    repository = actions.request('GET', '/repos/' + os.environ['GITHUB_REPOSITORY'])
    if not repository['private'] and not allow_public:
        raise ValueError('Caller is public: explicitly approve public input artifacts or use a private caller')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('poll', 'prepare', 'report'))
    parser.add_argument('--configuration-ref', default=os.environ.get('GITHUB_REF', ''))
    parser.add_argument('--allowlist', default='[]', help='JSON source-repository subset; opt-in is still required')
    parser.add_argument('--dry-run', action='store_true')
    parser.add_argument('--retry-failed', action='store_true')
    parser.add_argument('--allow-public-inputs', action='store_true')
    parser.add_argument('--request-id')
    parser.add_argument('--output', type=Path, default=Path('prepared'))
    args = parser.parse_args()
    github = GitHub(os.environ.get('SOURCE_CI_GITHUB_TOKEN'))
    actions = GitHub(os.environ.get('GITHUB_TOKEN'))
    repository, run_id, attempt = (os.environ[key] for key in ('GITHUB_REPOSITORY', 'GITHUB_RUN_ID', 'GITHUB_RUN_ATTEMPT'))
    s3 = boto3.client('s3')
    state = State(s3, os.environ['SOURCE_CI_STATE_URI'], repository)
    if args.command == 'poll':
        if not args.dry_run:
            artifact_policy(actions, args.allow_public_inputs)
            reconcile(state, github, actions)
        allowlist = json.loads(args.allowlist)
        if not isinstance(allowlist, list) or not all(isinstance(item, str) for item in allowlist):
            raise ValueError('Invalid source allowlist')
        configuration_sha = github.head(repository, args.configuration_ref.removeprefix('refs/heads/'))
        with tempfile.TemporaryDirectory(prefix='tailor-source-discovery-') as temporary:
            root = Path(temporary)
            github.archive(repository, configuration_sha, root / 'configuration.zip')
            extract_source(root / 'configuration.zip', root / 'configuration')
            requests = list(discover(root / 'configuration', repository, configuration_sha, args.configuration_ref,
                                     github, s3, allowlist))
        if args.dry_run:
            print(f'Discovered {len(requests)} source/OS requests; no statuses, state, or artifacts written')
            claimed = []
        else:
            claimed = enqueue(requests, state, github, actions, repository, run_id, attempt,
                              retry_failed=args.retry_failed)
        if os.environ.get('GITHUB_OUTPUT'):
            with open(os.environ['GITHUB_OUTPUT'], 'a') as output:
                print('matrix=' + json.dumps({'include': [{'id': key} for key in claimed]}), file=output)
                print('has_requests=' + str(bool(claimed)).lower(), file=output)
    elif args.command == 'prepare':
        artifact_policy(actions, args.allow_public_inputs)
        key, record, etag = owned(state, args.request_id)
        if record['state'] != 'queued':
            raise ValueError('Request is no longer queued')
        try:
            prepare(record['request'], github, s3, lambda region: boto3.client('ecr', region_name=region), args.output)
        except Exception:
            state.put(key, dict(record, state='preparation_failed'), etag)
            raise
        state.put(key, dict(record, state='prepared'), etag)
    else:
        manifest, _ = state.get(f'runs/{run_id}-{attempt}.json')
        if not manifest:
            return
        jobs = list(actions.pages(f'/repos/{repository}/actions/runs/{run_id}/jobs', 'jobs'))
        for request_id in manifest['requests']:
            key, record, etag = owned(state, request_id)
            finish(state, key, record, etag, jobs, github)


if __name__ == '__main__':
    main()
