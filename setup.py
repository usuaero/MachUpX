"""MachUpX: A new implementation of Phillips' numerical lifting-line algorigthm, combining the best of MachUp Pro and MachUp_Py"""

from setuptools import setup
import os
import sys

setup(name = 'MachUpX',
    version = '2.4.0',
    description = "MachUpX: A new implementation of Phillips' numerical lifting-line algorigthm, combining the best of MachUp Pro and MachUp_Py",
    url = 'https://github.com/usuaero/MachUpX',
    author = 'usuaero',
    author_email = 'doug.hunsaker@usu.edu',
    install_requires = ['numpy>=1.18', 'scipy>=1.4', 'pytest', 'matplotlib', 'numpy-stl', 'airfoil_db==1.4.1'],
    python_requires ='>=3.6.0',
    license = 'MIT',
    packages = ['machupX'],
    zip_safe = False)
