# setup.py: install script for robot manipulator (roman)
'''
to install roman and its dependencies for development work,
run this cmd from roman directory:
    pip install -e .
'''
import setuptools

with open('requirements.txt') as req_file:
    requirements = req_file.read().splitlines()

setuptools.setup(
    name="msr-roman",
    version="0.1",
    author = "The Microsoft Research Robotics Team",
    author_email="msrobotics@microsoft.com",
    url = "https://github.com/microsoft/roman",
    packages=setuptools.find_packages(),
    include_package_data=True,

    # the packages that our package is dependent on
    install_requires=requirements
)