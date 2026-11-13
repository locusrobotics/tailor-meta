"""GitHub API adapter; never expose its token to source-test workers."""
import json
from urllib.parse import quote
from urllib.request import Request, urlopen


class GitHub:
    def __init__(self, token):
        if not token:
            raise ValueError('Central GitHub token is required')
        self.token = token

    def request(self, method, path, data=None):
        request = Request('https://api.github.com' + path,
                          data=json.dumps(data).encode() if data is not None else None,
                          method=method, headers={'Authorization': 'Bearer ' + self.token,
                                                  'Accept': 'application/vnd.github+json',
                                                  'X-GitHub-Api-Version': '2022-11-28'})
        with urlopen(request, timeout=60) as response:
            body = response.read()
        return json.loads(body) if body else None

    def pages(self, path, field=None):
        separator = '&' if '?' in path else '?'
        for page in range(1, 101):
            result = self.request('GET', f'{path}{separator}per_page=100&page={page}')
            values = result[field] if field else result
            yield from values
            if len(values) < 100:
                return
        raise RuntimeError('GitHub pagination limit reached; narrow the polling scope')

    def head(self, repository, branch):
        return self.request('GET', f'/repos/{repository}/commits/{quote(branch, safe="")}')['sha']

    def status(self, request, state, description, url):
        self.request('POST', f'/repos/{request["repository"]}/statuses/{request["sha"]}',
                     {'state': state, 'context': request['context'], 'description': description[:140],
                      'target_url': url})

    def archive(self, repository, sha, destination):
        # GitHub's archive endpoint redirects to codeload. Do not forward the
        # installation/PAT authorization header to a different host.
        from urllib.request import HTTPRedirectHandler, build_opener

        class NoCredentialRedirect(HTTPRedirectHandler):
            def redirect_request(self, req, fp, code, msg, headers, newurl):
                from urllib.parse import urlsplit
                target = urlsplit(newurl)
                if target.scheme != 'https' or target.hostname != 'codeload.github.com':
                    raise ValueError('Unexpected archive redirect')
                return Request(newurl, headers={'Accept': 'application/zip'})

        request = Request(f'https://api.github.com/repos/{repository}/zipball/{sha}',
                          headers={'Authorization': 'Bearer ' + self.token})
        total = 0
        with build_opener(NoCredentialRedirect()).open(request, timeout=120) as response, destination.open('wb') as out:
            while chunk := response.read(1024 * 1024):
                total += len(chunk)
                if total > 512 * 1024 * 1024:
                    raise ValueError('Source archive exceeds pilot size limit')
                out.write(chunk)
