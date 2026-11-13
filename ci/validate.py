"""Offline configuration validation, safe to run on pull requests."""
import pathlib
import sys

import jinja2
import yaml

root = pathlib.Path(sys.argv[1])
for directory in ('rosdistro', 'rosdep', 'config'):
    for path in (root / directory).glob('*.yaml'):
        if path.name != 'upstream.yaml':
            yaml.safe_load(path.read_text())
recipes = yaml.safe_load((root / 'config/recipes.yaml').read_text())
template = jinja2.Environment(undefined=jinja2.StrictUndefined).from_string(
    (root / 'config/upstream.yaml').read_text())
for codename in recipes['os']['ubuntu']:
    yaml.safe_load(template.render(distribution=codename))
    print(f'Parsed upstream configuration for {codename}')
