#!/usr/bin/python3
import argparse
import click
import github
import pathlib
import rosdistro
import sys

from datetime import datetime, timezone
from time import sleep
from typing import Mapping, Any
from urllib.parse import urlsplit
from github.GithubException import UnknownObjectException, RateLimitExceededException

from . import YamlLoadAction


def gh_with_retry(client: github.Github, func, *args, **kwargs):
    try:
        func(*args, **kwargs)
    except RateLimitExceededException:
        reset = client.get_rate_limit().core.reset
        now = datetime.now(tz=timezone.utc)
        seconds = (reset - now).total_seconds() + 1
        click.echo(f"Rate limited by Github, waiting {seconds} seconds for point reset")
        sleep(seconds)

        click.echo(f"Points should be reset, remaining GH requests: {client.get_rate_limit().core.remaining}")
        click.echo(f"Trying request again...")

        func(*args, **kwargs)


def update_repo_settings(rosdistro_index: pathlib.Path, recipes: Mapping[str, Any],
                         github_key: str, deploy: bool, release_track: str):
    index = rosdistro.get_index(rosdistro_index.resolve().as_uri())
    github_client = github.Github(github_key)  # TODO(pbovbel) support more than just github?

    common_options = recipes['common']

    for distro_name, _ in common_options['distributions'].items():

        distro = rosdistro.get_distribution(index, distro_name)

        for repo_name, repository_data in distro.repositories.items():
            if not repository_data.source_repository or not repository_data.source_repository.test_commits:
                continue

            click.echo(f"Updating settings for repo {repo_name}", err=True)
            url = repository_data.source_repository.url

            gh_repo_name = urlsplit(url).path[len('/'):-len('.git')]
            try:
                gh_repo = github_client.get_repo(gh_repo_name, lazy=False)
            except UnknownObjectException:
                raise KeyError(
                    f"Please check if the repo exists or if permissions are restrictive"
                )


            # Remove wikis and projects
            if deploy:
                gh_with_retry(
                    github_client,
                    gh_repo.edit,
                    has_wiki=False,
                    has_projects=False
                )

            # Set PR merge to squash
            if deploy:
                gh_with_retry(
                    github_client,
                    gh_repo.edit,
                    allow_merge_commit=False,
                    allow_rebase_merge=False,
                    allow_squash_merge=True,
                    delete_branch_on_merge=True
                )

            # Protect branch
            branch = gh_repo.get_branch(repository_data.get_data()["source"]["version"])
            if deploy:
                gh_with_retry(
                    github_client,
                    branch.edit_protection,
                    strict=True,
                    required_approving_review_count=1
                )

            # Create label
            if deploy:
                try:
                    gh_repo.get_label(release_track)
                except UnknownObjectException:
                    gh_with_retry(
                        github_client,
                        gh_repo.create_label,
                        release_track,
                        color="00ff00"
                    )


def main():
    parser = argparse.ArgumentParser(description=update_repo_settings.__doc__)
    parser.add_argument('--rosdistro-index', type=pathlib.Path)
    parser.add_argument('--github-key', type=str)
    parser.add_argument('--recipes', action=YamlLoadAction, required=True)
    parser.add_argument('--release-track', type=str, required=True)
    parser.add_argument('--deploy', action='store_true')
    args = parser.parse_args()

    sys.exit(update_repo_settings(**vars(args)))


if __name__ == '__main__':
    main()
