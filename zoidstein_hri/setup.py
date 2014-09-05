#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['zoidstein_hri'],
    package_dir={'': 'src'}
)

setup(**d)