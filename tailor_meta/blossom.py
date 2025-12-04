import argparse
import apt
import jinja2
import os
import stat
import re
import yaml
import subprocess

from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import (
    List,
    Dict,
    Any,
    Tuple,
)
from catkin_pkg.topological_order import topological_order
from catkin_pkg.package import Package
from rosdep2.sources_list import SourcesListLoader
from rosdep2.lookup import RosdepLookup, ResolutionError
from rosdep2.rospkg_loader import DEFAULT_VIEW_KEY

from . import debian_templates
from .recipes import load_recipes, GlobalRecipe

TEMPLATE_SUFFIX = ".j2"
SCHEME_S3 = "s3://"
APT_REGION = os.environ.get("LOCUS_APT_REGION", "us-east-1")
APT_REPO = os.environ.get("LOCUS_APT_REPO", "s3://locus-tailor-artifacts")

@dataclass
class GraphPackage:
    name: str
    version: str
    sha: str
    path: str
    apt_depends: List[str]
    source_depends: List[str]
    reverse_depends: List[str] = field(default_factory=list)
    apt_candidate_version: str | None = None

    def __hash__(self):
        return hash(self.path)

    def debian_name(self, organization: str, release_label: str, distribution: str):
        return f"{organization}-{release_label}-{distribution}-{self.name.replace('_', '-')}"

    def debian_version(self, build_date: str):
        return f"{self.version}-{build_date}+git{self.sha}"

@dataclass
class Graph:
    """
    Class to represent a graph of all packages. This has convenience functions
    for generating a yaml representation of this graph, reading from an existing
    yaml, and generating
    """
    os_name: str
    os_version: str
    distribution: str
    release_label: str
    packages: Dict[str, GraphPackage] = field(default_factory=dict)
    organization: str = "locusrobotics"

    def add_package(self, package: Package, path: Path, conditions: Dict[str, Any] = {}):
        if package.name in self.packages:
            raise Exception(f"{package.name} already exists in the graph!")

        folder = path.parts[1]
        sha = folder.split("-")[-1][:7]

        dependencies = [
            dep.name for dep in
            package.build_export_depends + package.buildtool_export_depends +
            package.exec_depends + package.build_depends + package.doc_depends +
            package.exec_depends + package.buildtool_depends + package.test_depends
            if dep.evaluate_condition(conditions) or dep.evaluated_condition
        ]

        apt_deps = set()
        source_deps = set()

        for dep in dependencies:
            try:
                definition = self._rosdep_view.lookup(dep)

                rules = definition.get_rule_for_platform(
                    os_name=self.os_name,
                    os_version=self.os_version,
                    installer_keys=["apt"],
                    default_installer_key="apt"
                )

                for apt_pkg in rules[1]:
                    # TODO: Sometimes rosdep returns strange package names which
                    #       don't appear to exist.
                    #       e.g. libboost-filesystem resolves to libboost-filesystem1.74.0
                    #if apt_pkg not in self._apt_cache:
                    #    print(f"original dep: {dep}")
                    #    print(rules)
                    #    print(apt_pkg)
                    #    raise Exception(f"Missing APT dependency {apt_pkg}")
                    apt_deps.add(apt_pkg)

            except (KeyError, ResolutionError):
                source_deps.add(dep)

        pkg = GraphPackage(
            package.name,
            package.version,
            sha,
            str(path),
            list(apt_deps),
            list(source_deps)
        )

        # Check if there is an APT candidate for the source package
        try:
            deb_name = pkg.debian_name(self.organization, self.release_label, self.distribution)
            pkg.apt_candidate_version = self._apt_cache[deb_name].candidate.version
        except KeyError:
            pass

        self.packages[package.name] = pkg

        # Calculate reverse depends afterwards

    def finalize(self):
        for name, package in self.packages.items():
            for depend in package.source_depends:
                if depend not in self.packages:
                    raise Exception(f"Package {depend} was marked as a source dependency, but it was not found")

                if name not in self.packages[depend].reverse_depends:
                    self.packages[depend].reverse_depends.append(name)

    def _recurse_depends(self, depend: str, visited=None, rdeps=False, apt_depends=False):
        dep_type = "reverse_depends" if rdeps else "source_depends"

        if apt_depends:
            dep_type = "apt_depends"

        if visited is None:
            visited = set()

        if depend in visited:
            return set()

        visited.add(depend)
        d = set()

        for dep in getattr(self.packages[depend], dep_type):
            d.add(dep)
            d.update(self._recurse_depends(dep, visited, rdeps=rdeps))

        return d

    def debian_source_depends(self, package: GraphPackage, build_list: Dict[str, GraphPackage], build_date: str):
        depends = set()

        # APT depends can be set as-is
        for dep in package.apt_depends:
            depends.add(dep)

        # Source depends need to be handled in a way that we can re-use existing
        # APT candidates, but also include any new versions being built in this run
        # hence the need to pass a build list.
        for dep in package.source_depends:
            dep_pkg = self.packages[dep]
            deb_name = dep_pkg.debian_name(self.organization, self.release_label, self.distribution)

            if dep not in build_list:
                # This dependency is not being rebuilt, reuse existing package
                deb_version = dep_pkg.apt_candidate_version
            else:
                # Generate a new version
                deb_version = dep_pkg.debian_version(build_date)

            depends.add(f"{deb_name} (= {deb_version})")

        return list(depends)

    def all_source_depends(self, package: str, include_apt=False) -> List[str]:
        deps = self._recurse_depends(package, rdeps=False)

        return list(deps)

    def all_source_rdepends(self, package: str, include_apt=False) -> List[str]:
        rdeps = self._recurse_depends(package, rdeps=True)

        return list(rdeps)

    def get_depends(self, package: str) -> Tuple[List[str], List[str]]:
        return self.packages[package].apt_depends, self.packages[package].source_depends

    def all_upstream_depends(self, package: str):



        return self.packages[package].apt_depends

    def package_needs_rebuild(self, package: GraphPackage) -> bool:
        # Rebuild if no candidate was found
        if not package.apt_candidate_version:
            return True

        # Otherwise check if the candidates git SHA matches what we have cloned
        deb_name = package.debian_name(self.organization, self.release_label, self.distribution)
        sha = package.apt_candidate_version.split("+git")[-1][:7]
        if sha == package.sha:
            print(f"{package.name} has already been built ({deb_name}={package.apt_candidate_version})")
            return False

        print(f"Previously built {package.name} SHA {sha} does not match {package.sha}, need to rebuild")

        return True

    def build_list(self, root_packages: List[str], skip_rdeps: bool = False) -> Dict[str, GraphPackage]:
        """
        From an initial list of packages collect all dependent packages that
        don't already have a build candidate. If a package needs to be rebuilt
        this will also trigger any reverse dependencies to also be re-built.
        """
        build_list: Dict[str, GraphPackage] = {}

        def add_rdeps(name: str):
            rdeps = self.all_source_rdepends(name)

            # If this package is being rebuilt all reverse depends need to
            # also be rebuilt.
            for r in rdeps:
                if r in build_list:
                    continue

                build_list[r] = self.packages[r]

        if root_packages == []:
            # No packages specified, rebuild all
            root_packages = list(self.packages.keys())

        print(f"Generating list of packages to build... {root_packages}")

        for name in root_packages:
            package = self.packages[name]

            # Top level packages. If any need to be rebuilt also add rdeps
            if self.package_needs_rebuild(package):
                build_list[name] = package

                if not skip_rdeps:
                    add_rdeps(package.name)

            # Iterate the entire dependency tree, including nested dependencies
            for dep in self.all_source_depends(name):
                dep_pkg = self.packages[dep]

                if self.package_needs_rebuild(dep_pkg):
                    build_list[dep] = self.packages[dep]

                    if not skip_rdeps:
                        add_rdeps(dep)

        return build_list

    def __post_init__(self):
        self._apt_cache = apt.Cache()
        sources_loader = SourcesListLoader.create_default()
        self._rosdep_lookup = RosdepLookup.create_from_rospkg(
            sources_loader=sources_loader
        )
        self._rosdep_view = self._rosdep_lookup.get_rosdep_view(DEFAULT_VIEW_KEY)

    def write_yaml(self, path: Path):
        if not path.exists():
            path.mkdir(exist_ok=True)

        filename = path / Path(self.name + ".yaml")

        with open(filename, "w") as f:
            yaml.safe_dump(asdict(self), f)

        print(f"Wrote {filename}")

    @classmethod
    def from_yaml(cls, file: Path):
        data = yaml.safe_load(file.read_text())

        packages: Dict[str, GraphPackage] = {}
        for name, pkg_data in data["packages"].items():
            packages[name] = GraphPackage(**pkg_data)

        data.pop("packages")

        graph = Graph(**data, packages=packages)
        graph.finalize()

        return graph

    @classmethod
    def from_recipes(cls, recipe_dir: Path, workspace: Path) -> List[Any]:
        recipes = load_recipes(recipe_dir)
        graphs = []

        for recipe in recipes:
            for distribution in recipe.distributions.keys():
                graph = Graph(recipe.os_name, recipe.os_version, distribution, recipe.release_label)
                print(f"Generating graph for {graph.name}")

                for path, package in topological_order(
                    workspace / Path("src") / Path(distribution)
                ):
                    graph.add_package(package, Path(path), conditions=recipe.distributions[distribution].env)

                graph.finalize()

                graphs.append(graph)

        return graphs

    @property
    def name(self):
        return f"{self.os_name}-{self.os_version}-{self.distribution}-graph"

@dataclass
class JenkinsJob:
    name: str
    path: str
    depends: List[str]

@dataclass
class DebianGenerator:
    recipe: GlobalRecipe
    graph: Graph

    def generate(self, workspace: Path, packages: List[str] = [], skip_rdeps: bool = False) -> List[JenkinsJob]:
        jobs: List[JenkinsJob] = []

        if packages == []:
            packages = list(self.recipe.root_packages[self.graph.distribution])

        build_list = self.graph.build_list(packages, skip_rdeps=skip_rdeps)

        for pkg in sorted(build_list.keys()):
            print(pkg)

        print("Generating debian templates:")
        for name, pkg in build_list.items():
            self.write_templates(pkg, build_list, workspace)

            deb_name = pkg.debian_name(self.graph.organization, self.graph.release_label, self.graph.distribution)

            depends = [
                dep.debian_name(self.graph.organization, self.graph.release_label, self.graph.distribution)
                for dep in build_list.values() if dep.name in pkg.source_depends
            ]

            print(f"Dependencies for {name}: {depends}")

            job = JenkinsJob(
                deb_name,
                pkg.path,
                depends
            )

            jobs.append(job)

        return jobs

    def write_templates(self, package: GraphPackage, build_list: Dict[str, GraphPackage], workspace: Path):
        """Create templates for debian build"""
        env = jinja2.Environment(
            loader=jinja2.PackageLoader("tailor_meta", "debian_templates"),
            undefined=jinja2.StrictUndefined,
            trim_blocks=True,
        )
        env.filters["regex_replace"] = lambda s, find, replace: re.sub(find, replace, s)
        env.filters["union"] = lambda left, right: list(set().union(left, right))

        for template_name in env.list_templates():
            if not template_name.endswith(TEMPLATE_SUFFIX):
                continue

            template_path = Path(debian_templates.__file__).parent / template_name
            output_path = (
                workspace / Path("src") / Path(self.graph.distribution) / package.path / Path("debian") / template_name[: -len(TEMPLATE_SUFFIX)]
            )

            output_path.parent.mkdir(parents=True, exist_ok=True)

            source_depends = self.graph.debian_source_depends(package, build_list, self.recipe.build_date)

            context = dict(
                package_name=package.name,
                distro_name=self.graph.distribution,
                debian_name=package.debian_name(self.recipe.organization, self.recipe.release_label, self.graph.distribution),
                debian_version=package.debian_version(self.recipe.build_date),
                run_depends=package.apt_depends + source_depends,
                src_dir=os.path.abspath(workspace / "src"),
                bucket_name=APT_REPO[len(SCHEME_S3) :],
                bucket_region=APT_REGION,
                os_version=self.recipe.os_version,
                organization=self.recipe.organization,
                release_label=self.recipe.release_label,
                cxx_flags=self.recipe.cxx_flags,
                cxx_standard=self.recipe.cxx_standard,
                python_version="3",
                os_name=self.graph.os_name,
            )

            template = env.get_template(template_name)
            stream = template.stream(**context)
            print(f"Writing {output_path} ...")
            stream.dump(str(output_path))

            current_permissions = stat.S_IMODE(os.lstat(template_path).st_mode)
            os.chmod(output_path, current_permissions)

    def __str__(self):
        return f"""
Package: {self.name}
Version: {self.version}
Depends: {', '.join(self.apt_dependencies + self.source_dependencies)}
Distribution: {self.distribution}
"""

def find_recipe_from_graph(graph: Graph, recipes_dir: Path) -> GlobalRecipe:
    recipes = load_recipes(recipes_dir)

    for recipe in recipes:
        if recipe.os_name != graph.os_name:
            continue
        if recipe.os_version != graph.os_version:
            continue
        if recipe.release_label != graph.release_label:
            continue

        return recipe

    raise Exception("Could not find a recipe matching graph")


def main():
    parser = argparse.ArgumentParser("blossom")
    parser.add_argument("action")
    parser.add_argument("--workspace", type=Path)
    parser.add_argument("--recipe", type=Path)
    parser.add_argument("--graph", type=Path)
    parser.add_argument("--packages", nargs='+', type=str)
    parser.add_argument("--skip-rdeps", action='store_true')
    parser.add_argument("--ros-distro", nargs='+', type=str)

    args = parser.parse_args()

    if args.action == "graph":
        graphs = Graph.from_recipes(args.recipe, args.workspace)

        for graph in graphs:
            graph.write_yaml(args.workspace / Path("graphs"))

    elif args.action == "generate":
        graph = Graph.from_yaml(args.graph)
        recipe = find_recipe_from_graph(graph, args.recipe)

        generator = DebianGenerator(recipe, graph)
        generator.generate(args.workspace, packages=args.packages, skip_rdeps=True)

    elif args.action == "build":
        graph = Graph.from_yaml(args.graph)
        recipe = find_recipe_from_graph(graph, args.recipe)

        generator = DebianGenerator(recipe, graph)
        jobs = generator.generate(args.workspace, packages=args.packages, skip_rdeps=args.skip_rdeps)

        jenkins_yaml = {
            "packages": [asdict(j) for j in jobs]
        }

        job_path = args.workspace / Path("jobs")
        job_yaml = job_path / Path(f"{graph.os_name}-{graph.os_version}-{graph.distribution}.yaml")

        job_path.mkdir(exist_ok=True)

        with open(job_yaml, "w") as f:
            yaml.safe_dump(jenkins_yaml, f)

        print(f"Wrote {job_yaml}")

    elif args.action == "test":
        graph = Graph.from_yaml(args.graph)
        recipe = find_recipe_from_graph(graph, args.recipe)

        generator = DebianGenerator(recipe, graph)
        jobs = generator.generate(args.workspace, packages=args.packages, skip_rdeps=args.skip_rdeps)

        for job in jobs:
            if job.depends == []:
                print(f"{job.name} has no deps!")

        done = []
        pending = []

        def deps_met(job: JenkinsJob):
            if job.name == "locusrobotics-build-per-package-ros1-rosmake":
                print(f"Checking {job.name}, deps={job.depends}")
                print(f"Done={done}")

            for dep in job.depends:
                if dep not in done:
                    return False

            return True

        while len(jobs) > 0:
            for job in jobs.copy():
                if deps_met(job):
                    print(f"Adding {job.name} to pending list")
                    pending.append(job)
                    jobs.remove(job)
                else:
                    #print(f"{job.name} has unmet dependencies: {job.depends}")
                    pass

            if len(pending) == 0:
                print("No more jobs to run")
                return

            current = pending.pop(0)

            #print(f"Pretending to build: {current.name}")

            subprocess.run(
                ['fakeroot', 'debian/rules', 'binary'],
                cwd=args.workspace / Path("src") / Path(graph.distribution) / current.path,
                check=True,
            )

            done.append(current.name)
    elif args.action == "install":

        install_list = []

        graph = Graph.from_yaml(args.graph)
        recipe = find_recipe_from_graph(graph, args.recipe)
        for pkg in args.packages:
            upstream, source = graph.get_depends(pkg)
            #print(f"Upstream deps: {upstream}")
            #print(f"Source deps: {source}")
            install_list.extend(upstream)
            install_list.extend(source)

        print(" ".join(install_list))


if __name__ == "__main__":
    main()
