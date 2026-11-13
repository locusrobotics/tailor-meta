"""Discover manifest-opted source revisions against a published environment."""
import json
from pathlib import Path
import re

import yaml

from tailor_meta.actions import release
from tailor_meta.create_pipelines import collect_sources
from .inputs import identifier


def ecr_location(registry):
    value = registry.removeprefix('https://')
    match = re.fullmatch(r'(\d{12})\.dkr\.ecr\.([a-z0-9-]+)\.amazonaws\.com/([a-z0-9/_-]+)', value)
    if not match:
        raise ValueError('Central pilot supports configured private AWS ECR registries only')
    return match.groups()


def environment(root, ref, s3):
    target = release(ref)
    if target['kind'] == 'trivial':
        raise ValueError('Configuration ref must select a build/release track')
    recipes = yaml.safe_load((root / 'config/recipes.yaml').read_text())
    common = recipes['common']
    bucket = common['apt_repo'].removeprefix('s3://')
    if bucket == common['apt_repo']:
        raise ValueError('Published environment must use the configured S3 repository')
    published = json.loads(s3.get_object(Bucket=bucket, Key=target['label'] + '/build-plan.json')['Body'].read())
    if published['label'] != target['label'] or 'images' not in published['completed']:
        raise ValueError('Source CI requires a successfully published image environment')
    if not re.fullmatch(r'\d{8}\.\d{6}', published['timestamp']):
        raise ValueError('Invalid published environment timestamp')
    ecr_location(common['docker_registry'])
    images = {}
    for version in recipes['os']['ubuntu']:
        image = published.get('test_images', {}).get(version, '')
        prefix = common['docker_registry'].removeprefix('https://') + '@'
        if not image.startswith(prefix) or not re.fullmatch(r'sha256:[a-f0-9]{64}', image.removeprefix(prefix)):
            raise ValueError('Republish the environment with pinned test-image digests before source CI')
        images[version] = image
    return recipes, target, published, images


def discover(root, configuration_repository, configuration_sha, ref, github, s3, allowlist):
    recipes, target, published, images = environment(root, ref, s3)
    grouped = collect_sources(root / 'rosdistro/index.yaml', recipes, '', target['track'], target['label'])
    for (repository, branch), contexts in sorted(grouped.items()):
        if allowlist and repository not in allowlist:
            continue
        head = github.head(repository, branch)
        candidates = [{'sha': head, 'branch': branch, 'pull_request': None}]
        from urllib.parse import quote
        for pr in github.pages(f'/repos/{repository}/pulls?state=open&base={quote(branch, safe="")}'):
            # Forks and untrusted contributors require a future explicit approval
            # path; never automatically release private inputs to them.
            if (pr['head']['repo'] and pr['head']['repo']['full_name'] == repository
                    and pr.get('author_association') in ('OWNER', 'MEMBER', 'COLLABORATOR')):
                candidates.append({'sha': pr['head']['sha'], 'branch': branch, 'pull_request': pr['number']})
        for context in contexts:
            namespace = context['rosdistro_name']
            records = [record for path, values in published['sources'].items()
                       if Path(path).name == namespace + '_repositories_data.jsonl' for record in values]
            baseline = next((record['sha'] for record in records if record['repo'] == context['repository_name']), None)
            if not baseline:
                raise ValueError(f'No published baseline source identity for {repository}/{namespace}')
            for candidate in candidates:
                for version in context['distributions']:
                    request = dict(candidate, repository=repository, repository_name=context['repository_name'],
                                   baseline_sha=baseline, distro=namespace, os=version,
                                   configuration_repository=configuration_repository, configuration_sha=configuration_sha,
                                   configuration_ref=ref, release_label=target['label'],
                                   published_configuration_sha=published['config_sha'],
                                   environment_timestamp=published['timestamp'], image=images[version],
                                   apt_bucket=recipes['common']['apt_repo'].removeprefix('s3://'),
                                   graph_key=f'{target["label"]}/graphs/{published["timestamp"]}/ubuntu-{version}-graph.yaml')
                    for key in ('sha', 'baseline_sha', 'configuration_sha', 'published_configuration_sha'):
                        if not re.fullmatch(r'[a-f0-9]{40}', request[key]):
                            raise ValueError('Source request contains an invalid commit identity')
                    family = {key: request[key] for key in ('repository', 'branch', 'pull_request', 'distro', 'os',
                                                            'configuration_repository', 'configuration_ref')}
                    request['family'] = identifier(family)
                    request['id'] = identifier(request)
                    context_name = '/'.join(('tailor', configuration_repository.split('/')[-1],
                                              target['label'], namespace, version))
                    request['context'] = (context_name if len(context_name) <= 100
                                          else context_name[:87] + '-' + identifier(context_name)[:12])
                    yield request
