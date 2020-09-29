from setuptools import setup

setup(
    name='roman',
    version='1.0',
    install_requires=[
        "numpy", "scipy", "pybullet", "gym"
    ],
    package_dir={'': 'src'})