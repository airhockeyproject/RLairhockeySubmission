from setuptools import find_packages
from setuptools import setup

setup(
    name='python',
    version='4.4.0',
    packages=find_packages(
        include=('python', 'python.*')),
)
