#!/usr/bin/python3
import argparse
import click
import github
import pathlib
import rosdistro
import sys
import yaml

from jinja2 import Environment, BaseLoader
from typing import Mapping, Any
from urllib.parse import urlsplit

from . import YamlLoadAction

JENKINSFILE_TEMPLATE = """#!/usr/bin/env groovy
@Library('tailor-meta@{{ tailor_meta }}')_
tailorTestPipeline(
  // Name of job that generated this test definition.
  rosdistro_job: '{{ rosdistro_job }}',
  // Distribution name
  rosdistro_name: '{{ rosdistro_name }}',
  // Release track to test branch against.
  release_track: '{{ release_track }}',
  // Release label to pull test images from.
  release_label: '{{ release_label }}',
  // OS distributions to test.
  distributions: ['{{ distributions | join("', '") }}'],
  // Version of tailor_meta to build against
  tailor_meta: '{{ tailor_meta }}',
  // Master or release branch associated with this track
  source_branch: '{{ source_branch }}',
  // Docker registry where test image is stored
  docker_registry: '{{ docker_registry }}'
)
"""


def create_pipelines(rosdistro_index: pathlib.Path, recipes: Mapping[str, Any], github_key: str, deploy: bool,
                     meta_branch: str, release_track: str, release_label: str, rosdistro_job: str):
    index = rosdistro.get_index(rosdistro_index.resolve().as_uri())
    github_client = github.Github(github_key)  # TODO(pbovbel) support more than just github?

    common_options = recipes['common']

    all_distros = [distro for distros in recipes['os'].values() for distro in distros]

    jenkins_jobs = []

    for distro_name, _ in common_options['distributions'].items():

        distro = rosdistro.get_distribution(index, distro_name)
        allowed_distros = common_options['distributions'][distro_name]['os']
        # If allowed distros is empty, because we used [] in recipes, use all distros
        if not allowed_distros:
            allowed_distros = all_distros

        for repo_name, repository_data in distro.repositories.items():
            if not repository_data.source_repository or not repository_data.source_repository.test_commits:
                continue

            click.echo(f"Managing Jenkinsfile for repo {repo_name}", err=True)

            url = repository_data.source_repository.url
            branch = repository_data.source_repository.version

            gh_repo_name = urlsplit(url).path[len('/'):-len('.git')]
            gh_repo = github_client.get_repo(gh_repo_name, lazy=False)

            context = {
                'tailor_meta': meta_branch,
                'source_branch': branch,
                'rosdistro_job': rosdistro_job,
                'rosdistro_name': distro_name,
                'release_track': release_track,
                'release_label': release_label,
                'distributions': allowed_distros,
                'docker_registry': common_options['docker_registry'],
            }

            new_jenkinsfile = Environment(loader=BaseLoader()).from_string(JENKINSFILE_TEMPLATE).render(**context)
            try:
                old_jenkinsfile = gh_repo.get_contents(path="/Jenkinsfile", ref=branch)

                if old_jenkinsfile.decoded_content.decode() != new_jenkinsfile:
                    click.echo("Updating existing file...", err=True)
                    if deploy:
                        gh_repo.update_file(
                            path='Jenkinsfile',
                            message='Tailor: Updating Jenkinsfile',
                            content=new_jenkinsfile,
                            sha=old_jenkinsfile.sha,
                            branch=branch)
                else:
                    click.echo(f"No change required", err=True)
            except github.GithubException:
                click.echo("Writing new file...", err=True)
                if deploy:
                    gh_repo.create_file(
                        path='Jenkinsfile',
                        message='Tailor: Creating Jenkinsfile',
                        content=new_jenkinsfile,
                        branch=branch)

            jenkins_jobs.append({'repo_name': repo_name, 'owner_name': gh_repo.owner.login})

    print(yaml.dump(jenkins_jobs))


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
