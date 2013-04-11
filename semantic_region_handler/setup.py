#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['semantic_region_handler'],
    package_dir={'':'src'},
    scripts=['scripts/circle_region_handler.py'],
#    requires=['actionlib','rospy_message_converter','world_msgs','worldlib']
)

setup(**d)