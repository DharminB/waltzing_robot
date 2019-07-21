#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_setup = generate_distutils_setup(
    packages=['waltzing_robot'],
    package_dir={'waltzing_robot': 'ros/src/waltzing_robot'}
)

setup(**package_setup)
