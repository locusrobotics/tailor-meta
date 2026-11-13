import argparse
import pathlib
import json

from datetime import datetime, timezone
from typing import List

from . import YamlLoadAction
from .blossom import Graph


def load_repositories(path):
    repos = {}
    with open(path, "r") as f:
        for line in f.readlines():
            info = json.loads(line)
            repos[info['repo']] = info["sha"]
        return repos


def generate_graphs(
    recipe: dict,
    workspace: pathlib.Path,
    release_label: str,
    build_date: str,
    apt_configs: List[pathlib.Path],
    skip_apt: bool,
    package_release_label: str | None = None,
):
    graphs: List[Graph] = Graph.from_recipe(
        recipe,
        workspace,
        release_label,
        build_date,
        apt_configs=apt_configs,
        init_apt=(not skip_apt),
        package_release_label=package_release_label,
    )

    for graph in graphs:
        graph.write_yaml(workspace / pathlib.Path("graphs"))

def main():
    parser = argparse.ArgumentParser(description="Create package graph(s) based on a recipe")
    parser.add_argument("--recipe", action=YamlLoadAction, required=True)
    parser.add_argument("--release-label", required=True)
    parser.add_argument("--workspace", type=pathlib.Path, required=True)
    parser.add_argument("--timestamp", type=str, default=datetime.now(timezone.utc).strftime("%Y%m%d.%H%M%S"))
    parser.add_argument("--apt-configs", nargs="+", type=pathlib.Path, default=[])
    parser.add_argument("--skip-apt", action="store_true")
    parser.add_argument(
        "--package-release-label",
        type=str,
        default=None,
        help="Override used only for Debian package names. Defaults to --release-label.",
    )
    args = parser.parse_args()

    generate_graphs(
        args.recipe,
        args.workspace,
        args.release_label,
        args.timestamp,
        args.apt_configs,
        args.skip_apt,
        args.package_release_label,
    )


if __name__ == '__main__':
    main()
