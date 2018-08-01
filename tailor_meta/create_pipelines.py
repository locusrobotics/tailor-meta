#!/usr/bin/python3
import argparse
import sys


def create_pipelines(deploy):
    pass


def main():
    parser = argparse.ArgumentParser(description=create_pipelines.__doc__)
    # parser.add_argument('upstream_template', type=argparse.FileType('r'))
    # parser.add_argument('--version', type=str, required=True)
    # parser.add_argument('--release-track', type=str, required=True)
    # parser.add_argument('--endpoint', type=str, required=True)
    # parser.add_argument('--distribution', type=str, required=True)
    # parser.add_argument('--keys', type=pathlib.Path, nargs='+')
    parser.add_argument('--deploy', action='store_true')
    args = parser.parse_args()

    sys.exit(create_pipelines(**vars(args)))


if __name__ == '__main__':
    main()
