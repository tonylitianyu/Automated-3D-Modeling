from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup(
    packages=['camera_motion'],
    package_dir={'': 'src'}
    )
setup(**d)
