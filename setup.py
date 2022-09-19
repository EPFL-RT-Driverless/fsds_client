from setuptools import setup, find_packages

with open("README.md", "r") as fh:
    long_description = fh.read()

with open("requirements.txt", "r") as fh:
    requirements = fh.read().splitlines()
    # remove lines that start with #
    requirements = [r for r in requirements if not r.startswith("#")]

setup(
    name="fsds_client",
    version="2.0.0",
    description="Simple Python client for FSDS simulation",
    long_description=long_description,
    url="https://github.com/EPFL-RT-Driverless/fsds_client",
    author="Mattéo Berthet",
    license="MIT",
    classifiers=[
        # "Development Status :: 1 - Planning",
        # "Development Status :: 2 - Pre-Alpha",
        # "Development Status :: 3 - Alpha",
        "Development Status :: 4 - Beta",
        # "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Private :: Do Not Upload",
    ],
    packages=find_packages(include=["fsds_client"]),
    install_requires=requirements,
)
