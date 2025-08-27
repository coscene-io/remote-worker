from setuptools import find_packages
from setuptools import setup

setup(
    name='roarm_moveit',
    version='0.3.0',
    packages=find_packages(
        include=('roarm_moveit', 'roarm_moveit.*')),
)
