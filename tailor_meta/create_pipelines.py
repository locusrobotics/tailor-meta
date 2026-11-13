#!/usr/bin/python3
import argparse
import json
import click
import github
import pathlib
import rosdistro
import sys
import yaml

from typing import Mapping, Any
from urllib.parse import urlsplit

from . import YamlLoadAction

WORKFLOW_PATH = '.github/workflows/tailor.yaml'


def source_workflow(context):
    """Only a caller stub is installed in each opted-in source repository."""
    return yaml.safe_dump({
        'name': 'Tailor source checks',
        'on': {'push': {'branches': [context['source_branch']]}, 'pull_request': {}, 'workflow_dispatch': {
            'inputs': {'source_ref': {'description': 'Optional reviewed PR ref (pull/123/merge)',
                                      'type': 'string', 'default': ''}},
        }},
        'permissions': {'contents': 'read'},
        'jobs': {'tailor': {
            'uses': f"locusrobotics/tailor-meta/.github/workflows/source.yaml@{context['tailor_meta']}",
            'with': {
                'tailor_meta': context['tailor_meta'],
                'rosdistro_name': context['rosdistro_name'],
                'repository_name': context['repository_name'],
                'source_branch': context['source_branch'],
                'source_ref': '${{ inputs.source_ref || github.sha }}',
                'release_track': context['release_track'],
                'release_label': context['release_label'],
                'distributions': json.dumps(context['distributions']),
                'docker_registry': context['docker_registry'],
            },
            'secrets': 'inherit',
        }},
    }, sort_keys=False)


def create_pipelines(rosdistro_index: pathlib.Path, recipes: Mapping[str, Any], github_key: str, deploy: bool,
                     meta_branch: str, release_track: str, release_label: str, rosdistro_job: str):
    index = rosdistro.get_index(rosdistro_index.resolve().as_uri())
    github_client = github.Github(github_key)  # TODO(pbovbel) support more than just github?

    common_options = recipes['common']

    all_distros = [distro for distros in recipes['os'].values() for distro in distros]

    managed_repositories = []

    for distro_name, _ in common_options['distributions'].items():

        distro = rosdistro.get_distribution(index, distro_name)
        try:
            allowed_distros = common_options['distributions'][distro_name]['os']
            # If allowed distros is empty, because we used [] in recipes, use all distros
            if not allowed_distros:
                allowed_distros = all_distros
        except KeyError:
            # If ros distro doesn't have distros configured, use all distros
            allowed_distros = all_distros

        for repo_name, repository_data in distro.repositories.items():
            if not repository_data.source_repository or not repository_data.source_repository.test_commits:
                continue

            click.echo(f"Managing Actions workflow for repo {repo_name}", err=True)

            url = repository_data.source_repository.url
            branch = repository_data.source_repository.version

            gh_repo_name = urlsplit(url).path.strip('/').removesuffix('.git')
            gh_repo = github_client.get_repo(gh_repo_name, lazy=False)

            context = {
                'tailor_meta': meta_branch,
                'repository_name': repo_name,
                'source_branch': branch,
                'rosdistro_job': rosdistro_job,
                'rosdistro_name': distro_name,
                'release_track': release_track,
                'release_label': release_label,
                'distributions': allowed_distros,
                'docker_registry': common_options['docker_registry'],
            }

            new_workflow = source_workflow(context)
            try:
                old_workflow = gh_repo.get_contents(path=WORKFLOW_PATH, ref=branch)

                if old_workflow.decoded_content.decode() != new_workflow:
                    click.echo("Updating existing file...", err=True)
                    if deploy:
                        gh_repo.update_file(
                            path=WORKFLOW_PATH,
                            message='Tailor: Updating Actions workflow',
                            content=new_workflow,
                            sha=old_workflow.sha,
                            branch=branch)
                else:
                    click.echo("No change required", err=True)
            except github.GithubException as error:
                if error.status != 404:
                    raise
                click.echo("Writing new file...", err=True)
                if deploy:
                    gh_repo.create_file(
                        path=WORKFLOW_PATH,
                        message='Tailor: Creating Actions workflow',
                        content=new_workflow,
                        branch=branch)

            managed_repositories.append({'repo_name': repo_name, 'owner_name': gh_repo.owner.login})

    print(yaml.dump(managed_repositories))


def main():
    parser = argparse.ArgumentParser(description=create_pipelines.__doc__)
    parser.add_argument('--rosdistro-index', type=pathlib.Path)
    parser.add_argument('--rosdistro-job', type=str)
    parser.add_argument('--github-key', type=str)
    parser.add_argument('--recipes', action=YamlLoadAction, required=True)
    parser.add_argument('--meta-branch', type=str, required=True)
    parser.add_argument('--release-track', type=str, required=True)
    parser.add_argument('--release-label', type=str, required=True)
    parser.add_argument('--deploy', action='store_true')
    args = parser.parse_args()

    sys.exit(create_pipelines(**vars(args)))


if __name__ == '__main__':
    main()
