#!/usr/bin/env python3

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(
    version='0.0.1',
    scripts=['scripts'],
    packages=['vehicle'],
    package_dir={'': 'src'}
)
