# # ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
#setup_args
e = generate_distutils_setup(
    packages=['coop_control'],
    package_dir={'': 'src'},
    requires=['std_msgs', 'roscpp']
)

setup(**#setup_args
e)
