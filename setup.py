import setuptools

setuptools.setup(
    name='roman',
    version='2.0',
    url = "https://github.com/Infinite-Blue-1042/roman",
    install_requires=[
        "numpy", "scipy", "pybullet", "gym", "tensorflow"
    ],
    packages=setuptools.find_packages(),
)
