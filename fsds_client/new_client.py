import warnings
from typing import Optional, Callable, Union

import msgpackrpc
import numpy as np

from .types import *
from .utils import *
from functools import cached_property
import track_database as tdb

import json
import os

__all__ = ["FSDSClient"]


class FSDSClient:
    """
    Notes:
        - we only consider a single car => development of a pipeline instead of a competition
    """

    # RPC client parameters
    ip: str
    port: int
    timeout_value: int
    rpc_client: msgpackrpc.Client

    # Vehicle info, internal buffers for each property containing the last received value
    car_name: str
    delta_max: float
    cones_range_limits: tuple[float]
    cones_bearing_limits: tuple[float]

    _data: dict[
        str,
        dict[
            str,
            Union[
                float,
                np.ndarray,
                KinematicsState,
                CarControls,
                ImageResponse,
                LidarData,
                ImuData,
                GpsData,
                GroundSpeedSensorData,
                WheelStates,
            ],
        ],
    ]

    _state: KinematicsState
    _yaw: float
    _control: CarControls
    _images: dict[str, ImageResponse]
    _point_cloud: LidarData
    _cones_observations: np.ndarray
    _all_cones: np.ndarray

    def __init__(
        self,
        ip="127.0.0.1",
        port=41451,
        timeout_value=3,
        api_control=True,
        default_car_name="FSCar",
        delta_max: float = np.deg2rad(45.0),
        cones_range_limits: Union[float, tuple[float]] = (0.0, 100.0),
        cones_bearing_limits: Union[float, tuple[float]] = (-np.pi, np.pi),
    ):
        # RPC client parameters
        self.ip = ip
        self.port = port
        self.timeout_value = timeout_value
        # initialize the client
        # noinspection PyTypeChecker
        self.rpc_client = None
        self._setup_client()
        self._try_until_success(self._confirm_connection, "Failed to ping " + self.ip)
        # Vehicle info
        with open(
            os.path.expanduser("~")
            + "/Formula-Student-Driverless-Simulator/settings.json"
        ) as f:
            settings = json.load(f)
            self._data = {
                car_name: {
                    sensor_name: None
                    for sensor_name in settings["Vehicles"][car_name]["Sensors"].keys()
                    if settings["Vehicles"][car_name]["Sensors"][sensor_name]["Enabled"]
                }
                | {
                    camera_name: ImageResponse()
                    for camera_name in settings["Vehicles"][car_name]["Cameras"].keys()
                }
                | {"yaw": 0.0}
                for car_name in settings["Vehicles"].keys()
            }
            assert (
                default_car_name in self._data.keys()
            ), "The main car name must be one of the following: " + ", ".join(
                self._data.keys()
            )
            self.default_car_name = default_car_name

        self.delta_max = delta_max
        self.cones_range_limits = (
            cones_range_limits
            if isinstance(cones_range_limits, tuple)
            else (0.0, cones_range_limits)
        )
        self.cones_bearing_limits = (
            cones_bearing_limits
            if isinstance(cones_bearing_limits, tuple)
            else (-cones_bearing_limits, cones_bearing_limits)
        )

        self.api_control = api_control
        self._state = KinematicsState()
        self._control = CarControls()
        self._point_cloud = LidarData()
        self._cones_observations = np.empty((0, 2))
        self._images = {}

        map_name = self.map_name
        assert (
            map_name in tdb.available_tracks
        ), f"Map {map_name} not found in track database"
        self.track = tdb.load_track(map_name)
        self._all_cones = np.vstack((self.track.right_cones, self.track.left_cones))

    @property
    def car_names(self):
        return list(self._data.keys())

    def _try_until_success(
        self,
        callback: Callable,
        error_message: str,
        aux_callback: Optional[Callable] = None,
        wait_delay=1.0,
    ):
        success = False
        result = None
        while not success:
            try:
                result = callback()
                success = True
            except Exception as e:
                warnings.warn(
                    f"[{type(self).__name__}] {error_message} | Error message: {e}"
                )
                if aux_callback:
                    aux_callback()
                sleep(wait_delay)
        return result

    def _try_and_fail(self, callback: Callable, error_message: str, raise_error=False):
        try:
            return callback()
        except Exception as e:
            warnings.warn(
                f"[{type(self).__name__}] {error_message} | Error message: {e}"
            )
            if raise_error:
                raise e

    def _setup_client(self):
        def set_rpc_client():
            self.rpc_client = msgpackrpc.Client(
                msgpackrpc.Address(self.ip, self.port),
                timeout=self.timeout_value,
                pack_encoding="utf-8",
                unpack_encoding="utf-8",
            )

        self._try_and_fail(
            set_rpc_client,
            "Failed to setup client",
        )

    def ping(self):
        return self.rpc_client.call("ping")

    def _confirm_connection(self):
        if not self.ping():
            raise ConnectionError("Failed to connect to FSDS server")

    def restart(self):
        self._try_and_fail(lambda: self.rpc_client.call("restart"), "Failed to reset")
        sleep(0.1)  # TODO: is this necessary?
        self._setup_client()

    @cached_property
    def map_name(self):
        """Returns the name of the current map, which does not change during the simulation lifetime."""
        # return self.rpc_client.call("getMap")
        return self._try_and_fail(
            lambda: self.rpc_client.call("getMap").removesuffix("_cones"),
            "Failed to get map name",
        )

    @property
    def api_control(self):
        return self.rpc_client.call("isApiControlEnabled", "FSCar")

    @api_control.setter
    def api_control(self, is_enabled):
        self._try_until_success(
            lambda: self.rpc_client.call("enableApiControl", is_enabled, self.car_name),
            "Failed to set API control to " + str(is_enabled),
            self.restart,  # if we do not restart, the same request will keep on failing
        )

    @property
    def state(self):
        def update_state():
            self._state = KinematicsState.from_msgpack(
                self.rpc_client.call("simGetGroundTruthKinematics", self.car_name)
            )

        self._try_and_fail(update_state, "Failed to get state")

        # position
        x = self._state.position.x_val
        y = self._state.position.y_val
        # orientation
        phi = (
            np.mod(to_eularian_angles(self._state.orientation)[2] + np.pi, 2 * np.pi)
            - np.pi
        )
        self._yaw = phi
        # linear velocity
        v = np.hypot(
            self._state.linear_velocity.x_val, self._state.linear_velocity.y_val
        )
        angle_vx_vy = np.arctan2(
            self._state.linear_velocity.y_val, self._state.linear_velocity.x_val
        )
        v_x = v * np.cos(angle_vx_vy - phi)
        v_y = v * np.sin(angle_vx_vy - phi)
        # angular velocity
        r = self._state.angular_velocity.z_val

        return np.array([x, y, phi, v_x, v_y, r])

    @property
    def control(self):
        return self._control

    @control.setter
    def control(self, control: np.ndarray):
        assert control.shape == (2,)
        np.clip(control[0], -1, 1, out=control[0])
        np.clip(control[1], -self.delta_max, self.delta_max, out=control[1])
        self._control = CarControls()
        self._control.throttle, self._control.brake = (
            (control[0], 0.0) if control[0] > 0 else (0.0, control[0])
        )
        self._control.steering = -control[1] / self.delta_max

        self._try_and_fail(
            lambda: self.rpc_client.call(
                "simSetVehicleControls", self.car_name, control
            ),
            "Failed to set control",
        )

    @property
    def cones_observations(self):
        # TODO: here do we just use the last state or do we call the API again?
        cartesian = (self._all_cones - self._state[:2]) @ np.array(
            [
                [np.cos(-self._yaw), -np.sin(-self._yaw)],
                [np.sin(-self._yaw), np.cos(-self._yaw)],
            ]
        ).T
        # transform to polar coordinates
        polar = np.hstack(
            (
                np.hypot(cartesian[:, 0], cartesian[:, 1])[:, np.newaxis],
                np.arctan2(cartesian[:, 1], cartesian[:, 0])[:, np.newaxis],
            )
        )
        # only keep the cones that are in front of the car and at less that 12m
        polar = polar[
            (self.cones_bearing_limits[0] <= polar[:, 1])
            & (polar[:, 1] <= self.cones_bearing_limits[1])
            & (self.cones_range_limits[0] <= polar[:, 0])
            & (polar[:, 0] <= self.cones_range_limits[1]),
            :,
        ]

        # add noise to the polar coordinates
        # polar += np.random.multivariate_normal(
        #     np.zeros(2),self.cones, size=polar.shape[0]
        # )

        self._cones_observations = polar
        return self._cones_observations

    @property
    def image(self):
        def get_image():
            self._image_responses = [
                ImageResponse.from_msgpack(response)
                for response in self.rpc_client.call(
                    "simGetImages",
                )
            ]
            img1d = np.fromstring(self.image.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape((self.image.height, self.image.width, 3))

        return self._try_and_fail(
            lambda: self.rpc_client.call(
                "simGetImage", self.camera_name, self.image_type
            ),
            "Failed to get image",
        )
