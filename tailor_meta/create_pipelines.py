"""Manage thin Actions caller stubs, aggregated by repository and source branch."""
import argparse
import json
import os
from pathlib import Path
from urllib.parse import urlsplit

import click
import github
import rosdistro
import yaml

from . import YamlLoadAction

WORKFLOW_PATH = '.github/workflows/tailor.yaml'


def source_workflow(contexts):
    """One workflow per branch, with a reusable job for every ROS distribution."""
    jobs = {}
    for context in contexts:
        key = context['rosdistro_name']
        if key in jobs:
            raise ValueError(f'Duplicate {key} source job on one repository branch')
        jobs[key] = {
            'uses': f'locusrobotics/tailor-meta/.github/workflows/source.yaml@{context["tailor_meta"]}',
            'with': {name: context[name] for name in (
                'rosdistro_name', 'repository_name', 'source_branch', 'release_track', 'release_label', 'docker_registry')},
            'secrets': 'inherit',
        }
        jobs[key]['with'].update(source_ref='${{ inputs.source_ref || github.sha }}',
                                 distributions=json.dumps(context['distributions']))
    branches = sorted({context['source_branch'] for context in contexts})
    return yaml.safe_dump({
        'name': 'Tailor source checks',
        'on': {'push': {'branches': branches}, 'pull_request': {'branches': list(branches)}, 'workflow_dispatch': {
            'inputs': {'source_ref': {'description': 'Optional reviewed PR ref (pull/123/merge)',
                                      'type': 'string', 'default': ''}}}},
        'permissions': {'contents': 'read'}, 'jobs': jobs,
    }, sort_keys=False)


def collect_sources(rosdistro_index, recipes, meta_branch, release_track, release_label):
    index = rosdistro.get_index(rosdistro_index.resolve().as_uri())
    common = recipes['common']
    all_os = [os_version for versions in recipes['os'].values() for os_version in versions]
    grouped = {}
    for distro_name, options in common['distributions'].items():
        distribution = rosdistro.get_distribution(index, distro_name)
        allowed = options.get('os') or all_os
        for name, data in distribution.repositories.items():
            source = data.source_repository
            if not source or not source.test_commits:
                continue
            url = urlsplit(source.url)
            if url.hostname != 'github.com':
                raise ValueError(f'Unsupported source host for {name}')
            repository = url.path.strip('/').removesuffix('.git')
            grouped.setdefault((repository, source.version), []).append(dict(
                tailor_meta=meta_branch, repository_name=name, source_branch=source.version,
                rosdistro_name=distro_name, release_track=release_track, release_label=release_label,
                distributions=[os_version for os_version in all_os if os_version in allowed],
                docker_registry=common['docker_registry'],
            ))
    return grouped


def create_pipelines(rosdistro_index, recipes, github_key, deploy, meta_branch, release_track, release_label,
                     output_dir=None):
    grouped = collect_sources(rosdistro_index, recipes, meta_branch, release_track, release_label)
    client = github.Github(github_key)
    for (repository, branch), contexts in sorted(grouped.items()):
        workflow = source_workflow(contexts)
        if output_dir:
            # Separate filenames for branches without ambiguous slash replacement.
            import hashlib
            branch_id = hashlib.sha256(branch.encode()).hexdigest()[:12]
            path = output_dir / repository / branch_id / 'tailor.yaml'
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(workflow)
        click.echo(f'{repository}@{branch}: {", ".join(c["rosdistro_name"] for c in contexts)}', err=True)
        if not deploy:
            continue
        repo = client.get_repo(repository)
        try:
            old = repo.get_contents(WORKFLOW_PATH, ref=branch)
        except github.GithubException as error:
            if error.status != 404:
                raise
            repo.create_file(WORKFLOW_PATH, 'Tailor: Create Actions workflow', workflow, branch=branch)
        else:
            if old.decoded_content.decode() != workflow:
                repo.update_file(WORKFLOW_PATH, 'Tailor: Update Actions workflow', workflow, old.sha, branch=branch)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--rosdistro-index', type=Path, required=True)
    parser.add_argument('--github-key', default=os.environ.get('GITHUB_TOKEN'))
    parser.add_argument('--recipes', action=YamlLoadAction, required=True)
    parser.add_argument('--meta-branch', required=True)
    parser.add_argument('--release-track', required=True)
    parser.add_argument('--release-label', required=True)
    parser.add_argument('--deploy', action='store_true')
    parser.add_argument('--output-dir', type=Path)
    create_pipelines(**vars(parser.parse_args()))


if __name__ == '__main__':
    main()
