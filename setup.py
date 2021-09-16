from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup
from Cython.Build import cythonize

d = generate_distutils_setup(
    packages=['dr_hardware_tests'],
    package_dir={'': 'src'}
)

setup(**d)