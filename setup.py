# setup.py: install script for robot manipulator (roman)
'''
to install roman and its dependencies for development work,
run this cmd from roman directory:
    pip install -e .
'''
import setuptools

setuptools.setup(
    name='roman',
    version='1.0',
    url="https://github.com/microsoft/roman",
    install_requires=[
        "numpy", "scipy", "pybullet"
    ],
    packages=setuptools.find_packages(),
)