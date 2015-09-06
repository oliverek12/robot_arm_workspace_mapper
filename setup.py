#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['abb_workspace_mapper'],
    package_dir={'': 'scripts'}
)

setup(**d)
