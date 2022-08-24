from setuptools import setup, find_packages

setup(
    name="Simple Python client for FSDS simulation",
    version="1.0.0",
    description="Simple Python client for FSDS simulation",
    url="https://github.com/EPFL-RT-Driverless/fsds_client",
    author="Mattéo Berthet",
    license="MIT",
    classifiers=[
        # "Development Status :: 1 - Planning",
        # "Development Status :: 2 - Pre-Alpha",
        # "Development Status :: 3 - Alpha",
        # "Development Status :: 4 - Beta",
        "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Private :: Do Not Upload",
    ],
    packages=find_packages(include=["fsds_client"]),
    install_requires=["numpy", "msgpack-rpc-python"],
)
