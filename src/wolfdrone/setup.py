#!/usr/bin/env python
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(packages=["wolfdrone"],package_dir={"": "scripts"})
