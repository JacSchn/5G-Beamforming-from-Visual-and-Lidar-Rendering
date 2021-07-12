from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['visual_self_driving_lidar'],
    package_dir={'': 'lib'}
)


setup(**d)

