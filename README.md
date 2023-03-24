# FSDS Client
This package is a reimplementation of the original python client of 
[FSDS](https://fs-driverless.github.io/Formula-Student-Driverless-Simulator/v2.1.0/getting-started-with-python/).

It has a simpler and more streamlined interface and also supports the new features
introduced by the EPFL Racing Team in [their fork](https://github.com/EPFL-RT-Driverless/FSDS).
Interesting features include:

- send receive information of arbitrary any vehicle or sensor
- 6 DOF state (pose in global cartesian coordinates and velocities in local cartesian coordinates) for localization and control tasks
- restarting the simulation
- getting the map name

This repo is still young, so feel free to tell us if it fits the needs of your 
team, suggest improvements, or report bugs.

## Repo structure
There are three python modules included:
- `client.py`: the one that will actually be useful for the users (and therefore
   the only one imported in the `__init__.py`). It defines the `FSDSClient` class
   that wraps the RPC client sending requests to FSDS.
- `types.py`: defines the types used to make RPC requests to FSDS. Should only be
   used inside the definition of `FSDSClient`.
- `utils.py`: defines utility functions used inside `client.py` (e.g. the 
  quaternion conversions to yaw)

## API reference

