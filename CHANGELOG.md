# v4.0.3

unpinned matplotlib and scipy versions

# v4.0.2

fixed a bug in `FSDSClient.get_cones_observations()`

# v4.0.1

added some documentation

# v4.0.0

New client for ARWO

# v3.0.2

- added `COLCON_IGNORE` file to ignore this package in colcon builds (for brains repo)
- updated msgpack rpc dep to new package [rpc-msgpack](https://pypi.org/project/rpc-msgpack/)
  that is compatible with latest versions of tornado 
  ([msgpack-rpc-python](https://pypi.org/project/msgpack-rpc-python/) 
  was not), which is nice for other dependencies that might need tornado (like jupyter).

# v3.0.1

added `COLCON_IGNORE` file to ignore this package in colcon builds (for brains repo)

# v3.0.0
Added several new features (mostly based on compatibility with the latest
release of FSDS) and bug fixes:
- added support for custom maps loaded at runtime via the `track_database` v3.0.0
- added `get_map_name()` to get the current track name at runtime.
- added `get_wheels_speed()` the rpm of each wheel.
- added `find_cones()` ot replicate the lidar cone detection capability but with
  the actual cones position fetched from the CSV file of `track_database` to
  which we add artificial noise.

# v2.0.2

:bug: added `sleep_sub_ms()` call to avoid erros in `enableApiControl`

# v2.0.1

🐛 Fixed import of git deps in `setup.py`

# v2.0.0

updated everything to match `python_boilerplate` v2.0.1
