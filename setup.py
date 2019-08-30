"""MachUpX: A new implementation of Phillips' numerical lifting-line algorigthm, cobining the best of MachUp Pro and MachUp_Py"""

from setuptools import setup
import os

setup(name = 'MachUpX',
    version = '1.0.1',
    description = "MachUpX: A new implementation of Phillips' numerical lifting-line algorigthm, cobining the best of MachUp Pro and MachUp_Py",
    url = 'https://github.com/usuaero/MachUpX',
    author = 'usuaero',
    author_email = 'doug.hunsaker@usu.edu',
    install_requires = ['numpy', 'scipy', 'pytest', 'matplotlib', 'pandas'],
    python_requires ='>=3.6.0',
    license = 'MIT',
    packages = ['machupX'],
    zip_safe = False)
