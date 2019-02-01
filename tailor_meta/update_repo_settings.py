#!/usr/bin/python3
import argparse
import click
import github
import pathlib
import rosdistro
import sys

from typing import Mapping, Any

from . import YamlLoadAction


def update_repo_settings(rosdistro_index: pathlib.Path, recipes: Mapping[str, Any],
                         github_key: str, deploy: bool):
    index = rosdistro.get_index(rosdistro_index.resolve().as_uri())
    github_client = github.Github(github_key)  # TODO(pbovbel) support more than just github?

    common_options = recipes['common']

    for distro_name, _ in common_options['distributions'].items():

        distro = rosdistro.get_distribution(index, distro_name)

        for repo_name, repository_data in distro.repositories.items():
            if not repository_data.source_repository or not repository_data.source_repository.test_commits:
                continue

            click.echo(f"Updating settings for repo {repo_name}", err=True)

            repo = github_client.get_repo("locusrobotics/"+repo_name)

            # Remove wikis, issues and projects
            if deploy:
                repo_name.edit(has_wiki=False,
                               has_issues=False,
                               has_projects=False)

            # Set PR merge to squash
            if deploy:
                repo_name.edit(allow_merge_commit=False,
                               allow_rebase_merge=False,
                               allow_squash_merge=False)

            # Protect master and release/* branches
            for branch in repo.get_branches():
                if branch.name == "master" or "release/" in branch.name:
                    if deploy:
                        branch.edit_protection(strict=True, required_approving_review_count=1)


def main():
    parser = argparse.ArgumentParser(description=update_repo_settings.__doc__)
    parser.add_argument('--rosdistro-index', type=pathlib.Path)
    parser.add_argument('--github-key', type=str)
    parser.add_argument('--recipes', action=YamlLoadAction, required=True)
    parser.add_argument('--deploy', action='store_true')
    args = parser.parse_args()

    sys.exit(update_repo_settings(**vars(args)))


if __name__ == '__main__':
    main()
