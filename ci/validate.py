"""One offline validator for local checks and credential-free hosted PR checks."""
import argparse
from pathlib import Path

from jinja2 import StrictUndefined
from jinja2.sandbox import SandboxedEnvironment
import yaml


def validate(root):
    for directory in ('rosdistro', 'rosdep', 'config'):
        for path in (root / directory).glob('*.yaml'):
            if path.name != 'upstream.yaml':
                yaml.safe_load(path.read_text())
    recipes = yaml.safe_load((root / 'config/recipes.yaml').read_text())
    template = SandboxedEnvironment(undefined=StrictUndefined).from_string(
        (root / 'config/upstream.yaml').read_text())
    for codename in recipes['os']['ubuntu']:
        yaml.safe_load(template.render(distribution=codename))
        print(f'Parsed upstream configuration for {codename}')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root', type=Path)
    parser.add_argument('--mode', choices=('validate', 'graph', 'packages', 'release'), default='validate')
    args = parser.parse_args()
    validate(args.root)
