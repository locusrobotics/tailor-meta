#!/usr/bin/python3
import argparse
import click
import github
import pathlib
import rosdistro
import sys

from jinja2 import Environment, BaseLoader
from typing import Mapping, Any
from urllib.parse import urlsplit

from . import YamlLoadAction

JENKINSFILE_TEMPLATE = """#!/usr/bin/env groovy
@Library('tailor-meta@{{ meta_branch }}')_
tailorTestPipeline(
  rosdistro: '{{ rosdistro }}',
  release_track: '{{ release_track }}',
  distributions: ['{{ distributions | join("', '") }}'],
  flavour: '{{ flavour }}'
)
"""


def create_pipelines(rosdistro_index: pathlib.Path, recipes: Mapping[str, Any], github_key: str, deploy: bool,
                     meta_branch: str, release_track: str):
    index = rosdistro.get_index(rosdistro_index.resolve().as_uri())
    github_client = github.Github(github_key)  # TODO(pbovbel) support more than just github?

    common_options = recipes['common']

    for distro_name, _ in common_options['distributions'].items():

        distro = rosdistro.get_distribution(index, distro_name)

        for repo_name, repository_data in distro.repositories.items():
            if not repository_data.source_repository or not repository_data.source_repository.test_commits:
                continue
            click.echo(f"Managing Jenkinsfile for repo {repo_name}", err=True)
            url = repository_data.source_repository.url
            branch = repository_data.source_repository.version

            gh_repo_name = urlsplit(url).path[len('/'):-len('.git')]
            gh_repo = github_client.get_repo(gh_repo_name, lazy=False)

            context = {
                'meta_branch': meta_branch,
                'rosdistro': distro_name,
                'release_track': release_track,
                'distributions': [distro for distros in recipes['os'].values() for distro in distros],
                'flavour': 'dev',  # TODO(pbovbel) should dev bundle be hardcoded?
            }

            new_jenkinsfile = Environment(loader=BaseLoader()).from_string(JENKINSFILE_TEMPLATE).render(**context)
            try:
                old_jenkinsfile = gh_repo.get_file_contents(path="/Jenkinsfile", ref=branch)
                if old_jenkinsfile.decoded_content.decode() != new_jenkinsfile:
                    click.echo("Updating existing...")
                    if deploy:
                        gh_repo.update_file(
                            path='/Jenkinsfile',
                            message='Tailor: Updating Jenkinsfile',
                            content=new_jenkinsfile,
                            sha=old_jenkinsfile.sha,
                            branch=branch)
                else:
                    click.echo(f"No difference detected for repo {repo_name}")
            except github.GithubException:
                click.echo("Writing new...")
                if deploy:
                    gh_repo.create_file(
                        path='/Jenkinsfile',
                        message='Tailor: Creating Jenkinsfile',
                        content=new_jenkinsfile,
                        branch=branch)


def main():
    parser = argparse.ArgumentParser(description=create_pipelines.__doc__)
    parser.add_argument('--rosdistro-index', type=pathlib.Path)
    parser.add_argument('--github-key', type=str)
    parser.add_argument('--recipes', action=YamlLoadAction, required=True)
    parser.add_argument('--meta-branch', type=str, required=True)
    parser.add_argument('--release-track', type=str, required=True)
    parser.add_argument('--deploy', action='store_true')
    args = parser.parse_args()

    sys.exit(create_pipelines(**vars(args)))


if __name__ == '__main__':
    main()
