from setuptools import setup, find_packages

setup(
    name="gitpod_template",
    version="1.0.0",
    description="",
    url="",
    author="",
    license="MIT",
    classifiers=[
        "Development Status :: 1 - Planning",
        # "Development Status :: 2 - Pre-Alpha",
        # "Development Status :: 3 - Alpha",
        # "Development Status :: 4 - Beta",
        # "Development Status :: 5 - Production/Stable",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Private :: Do Not Upload",
    ],
    packages=find_packages(include=["gitpod_template"]),
    install_requires=["numpy"],
)
