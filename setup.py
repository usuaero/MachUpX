"""MachUpX: An implementation of the Goates-Hunsaker generalized numerical lifting-line algorithm."""

from setuptools import setup
import os
import sys

setup(name = 'MachUpX',
    version = '2.7.2',
    description = "MachUpX: An implementation of the Goates-Hunsaker generalized numerical lifting-line algorithm.",
    url = 'https://github.com/usuaero/MachUpX',
    author = 'usuaero',
    author_email = 'doug.hunsaker@usu.edu',
    install_requires = ['numpy>=1.18', 'scipy>=1.4', 'pytest', 'matplotlib', 'numpy-stl', 'airfoil_db>=1.4.3'],# @ git+https://github.com/usuaero/AirfoilDatabase.git#egg=airfoil_db-1.4.3'],
    python_requires ='>=3.6.0',
    license = 'MIT',
    packages = ['machupX'],
    zip_safe = False)
