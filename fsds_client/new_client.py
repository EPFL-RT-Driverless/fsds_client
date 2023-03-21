import json
import os
import time
import warnings
from functools import cached_property
from typing import Optional, Callable, Union, Type
from PIL import Image
import cv2
import io

import msgpackrpc
import numpy as np

import track_database as tdb
from .types import *
from .utils import *

__all__ = ["FSDSClient"]

# 1,4,5 represent other AirSim sensor types that are not relevant in FSDS
_SENSOR_TYPES = {
    2: ImuData,
    3: GpsData,
    6: LidarData,
    7: GroundSpeedSensorData,
    8: ImageResponse,
    9: WheelStates,
}


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
    _data_types: dict[
        str,
        dict[
            str,
            Type[
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
                ]
            ],
        ],
    ]
    _all_cones: np.ndarray

    def __init__(
        self,
        ip="127.0.0.1",
        port=41451,
        timeout_value=3,
        api_control=True,
        restart=True,
        default_car_name: Optional[str] = None,
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
        self._try_until_success(self.ping, "Failed to ping " + self.ip)
        if restart:
            self._try_until_success(self.restart, "Failed to restart the simulation")

        # load vehicle info from settings.json
        with open(
            os.path.expanduser("~")
            + "/Formula-Student-Driverless-Simulator/settings.json"
        ) as f:
            settings = json.load(f)
            self._data = {}
            self._data_types = {}
            for car_name in settings["Vehicles"].keys():
                self._data[car_name] = {}
                self._data_types[car_name] = {}
                for sensor_name in settings["Vehicles"][car_name]["Sensors"].keys():
                    if settings["Vehicles"][car_name]["Sensors"][sensor_name][
                        "Enabled"
                    ]:
                        self._data_types[car_name][sensor_name] = _SENSOR_TYPES[
                            settings["Vehicles"][car_name]["Sensors"][sensor_name][
                                "SensorType"
                            ]
                        ]
                        self._data[car_name][sensor_name] = self._data_types[car_name][
                            sensor_name
                        ]()

                for camera_name in settings["Vehicles"][car_name]["Cameras"].keys():
                    self._data_types[car_name][camera_name] = ImageResponse
                    self._data[car_name][camera_name] = ImageResponse()

                self._data_types[car_name]["state"] = KinematicsState
                self._data[car_name]["state"] = KinematicsState()
                self._data_types[car_name]["control"] = CarControls
                self._data[car_name]["control"] = CarControls()
                self._data_types[car_name]["yaw"] = float
                self._data[car_name]["yaw"] = 0.0
                self._data_types[car_name]["cones_observations"] = np.ndarray
                self._data[car_name]["cones_observations"] = np.empty((0, 2))

            if default_car_name is None:
                default_car_name = list(settings["Vehicles"].keys())[0]

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

        sleep(1.0)  # important
        self.set_api_control(
            api_control
        )  # we do it here because we need the car name, we use the default one here

        map_name = self.map_name
        assert (
            map_name in tdb.available_tracks
        ), f"Map {map_name} not found in track database"
        self.track = tdb.load_track(map_name)
        self._all_cones = np.vstack((self.track.right_cones, self.track.left_cones))

    def car_names(self) -> list[str]:
        return list(self._data.keys())

    def camera_names(self, car_name: Optional[str] = None) -> list[str]:
        if car_name is None:
            car_name = self.default_car_name
        return list(
            filter(
                lambda k: self._data_types[car_name][k] == ImageResponse,
                self._data[car_name].keys(),
            )
        )

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
                    f"[{type(self).__name__}] {error_message} | {type(e)}: {e}"
                )
                if aux_callback:
                    aux_callback()
                sleep(wait_delay)
        return result

    def _setup_client(self):
        self.rpc_client = msgpackrpc.Client(
            msgpackrpc.Address(self.ip, self.port),
            timeout=self.timeout_value,
            pack_encoding="utf-8",
            unpack_encoding="utf-8",
        )

    def ping(self) -> bool:
        return self.rpc_client.call("ping")

    def restart(self):
        self.rpc_client.call("restart")
        sleep(0.1)  # necessary
        self._setup_client()

    @cached_property
    def map_name(self) -> str:
        """Returns the name of the current map, which does not change during the simulation lifetime."""
        return self.rpc_client.call("getMap").removesuffix("_cones")

    def get_api_control(self, car_name: Optional[str] = None) -> bool:
        if car_name is None:
            car_name = self.default_car_name
        return self.rpc_client.call("isApiControlEnabled", car_name)

    def set_api_control(self, enabled: bool, car_name: Optional[str] = None):
        if car_name is None:
            car_name = self.default_car_name

        self._try_until_success(
            lambda: self.rpc_client.call("enableApiControl", enabled, car_name),
            "Failed to set API control",
            self.restart,
        )

    def get_state(self, car_name: Optional[str] = None) -> tuple[np.ndarray, np.uint64]:
        if car_name is None:
            car_name = self.default_car_name

        self._data[car_name]["state"] = KinematicsState.from_msgpack(
            self.rpc_client.call("simGetGroundTruthKinematics", car_name)
        )
        timestamp = time.time_ns()
        state = self._data[car_name]["state"]

        # position
        x = state.position.x_val
        y = state.position.y_val
        # orientation
        phi = (
            np.mod(to_eularian_angles(state.orientation)[2] + np.pi, 2 * np.pi) - np.pi
        )
        self._data[car_name]["yaw"] = phi
        # linear velocity
        v = np.hypot(state.linear_velocity.x_val, state.linear_velocity.y_val)
        angle_vx_vy = np.arctan2(
            state.linear_velocity.y_val, state.linear_velocity.x_val
        )
        v_x = v * np.cos(angle_vx_vy - phi)
        v_y = v * np.sin(angle_vx_vy - phi)
        # angular velocity
        r = state.angular_velocity.z_val

        return (
            np.array([x, y, phi, v_x, v_y, r]),
            np.uint64(timestamp),
        )

    def set_control(self, control: np.ndarray, car_name: Optional[str] = None):
        if car_name is None:
            car_name = self.default_car_name

        assert control.shape == (2,)
        control[0] = np.clip(control[0], -1, 1)
        control[1] = np.clip(control[1], -self.delta_max, self.delta_max)

        self._data[car_name]["control"] = CarControls()
        (
            self._data[car_name]["control"].throttle,
            self._data[car_name]["control"].brake,
        ) = (
            (control[0], 0.0) if control[0] > 0 else (0.0, control[0])
        )
        self._data[car_name]["control"].steering = -control[1] / self.delta_max

        self.rpc_client.call(
            "setCarControls", self._data[car_name]["control"], car_name
        )

    def get_cones_observations(
        self, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        if car_name is None:
            car_name = self.default_car_name

        # TODO: here do we just use the last state or do we call the API again?
        self.get_state(car_name)

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

        self._data[car_name]["cones_observations"] = polar
        return (
            self._data[car_name]["cones_observations"],
            self._data[car_name]["state"].time_stamp,
        )

    def get_image(
        self,
        camera_names: list[str] = [],
        car_name: Optional[str] = None,
        compressed: bool = False,
    ) -> list[tuple[np.ndarray, np.uint64]]:
        # if compressed:
        #     raise NotImplementedError("Compressed images not implemented yet")

        if car_name is None:
            car_name = self.default_car_name

        if len(camera_names) == 0:
            camera_names = self.camera_names(car_name)[0]

        for camera_name in camera_names:
            assert (
                camera_name in self._data[car_name]
                and self._data_types[car_name][camera_name] == ImageResponse
            ), f"Camera {camera_name} not found"

        responses = [
            ImageResponse.from_msgpack(response)
            for response in self.rpc_client.call(
                "simGetImages",
                [
                    ImageRequest(
                        camera_name=camera_name,
                        image_type=ImageType.Scene,
                        compress=compressed,
                        # pixels_as_float=compressed,
                    )
                    for camera_name in camera_names
                ],
                car_name,
            )
        ]
        for i, camera_name in enumerate(camera_names):
            self._data[car_name][camera_name] = responses[i]

        # cv2image = cv2.imdecode(
        #     np.frombuffer(responses[0].image_data_uint8, dtype=np.uint8), flags=0
        # )
        # arr = np.asarray(cv2image)
        # print(arr.shape)
        return [
            (
                np.frombuffer(response.image_data_uint8, dtype=np.uint8).reshape(
                    (response.height, response.width, 3)
                )
                if not compressed
                else np.asarray(
                    # cv2.imdecode(
                    #     np.frombuffer(response.image_data_uint8, dtype=np.uint8),
                    #     flags=0,
                    # ),
                    Image.open(io.BytesIO(response.image_data_uint8), formats=["PNG"]),
                ).reshape((response.height, response.width, 4))[:, :, :3],
                response.time_stamp,
            )
            for response in responses
        ]

    def get_point_cloud(
        self, lidar_name: str, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        if car_name is None:
            car_name = self.default_car_name
        assert (
            lidar_name in self._data[car_name].keys()
            and self._data_types[car_name][lidar_name] == LidarData
        ), f"LiDAR {lidar_name} not found"

        self._data[car_name][lidar_name] = LidarData.from_msgpack(
            self.rpc_client.call("getLidarData", lidar_name, car_name)
        )
        time_stamp = np.uint64(time.time_ns())
        return (
            self._data[car_name][lidar_name].point_cloud,
            time_stamp,
        )

    def get_imu_data(
        self, imu_name: str, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        """Get IMU data as (phi, ax, ay, r)"""
        if car_name is None:
            car_name = self.default_car_name

        assert (
            imu_name in self._data[car_name].keys()
            and self._data_types[car_name][imu_name] == ImuData
        ), f'IMU "{imu_name}" not found'

        self._data[car_name][imu_name] = ImuData.from_msgpack(
            self.rpc_client.call("getImuData", imu_name, car_name)
        )
        time_stamp = np.uint64(time.time_ns())
        return (
            np.array(
                [
                    np.mod(
                        to_eularian_angles(self._data[car_name][imu_name].orientation)[
                            2
                        ]
                        + np.pi,
                        2 * np.pi,
                    )
                    - np.pi,
                    self._data[car_name][imu_name].linear_acceleration.x_val,
                    self._data[car_name][imu_name].linear_acceleration.y_val,
                    self._data[car_name][imu_name].angular_velocity.z_val,
                ]
            ),
            time_stamp,
        )

    def get_gss_data(
        self, gss_name: str, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        """Get GSS data as (vx, vy)"""
        if car_name is None:
            car_name = self.default_car_name

        assert (
            gss_name in self._data[car_name].keys()
            and self._data_types[car_name][gss_name] == GroundSpeedSensorData
        ), f"GSS {gss_name} not found"

        self._data[car_name][gss_name] = GroundSpeedSensorData.from_msgpack(
            self.rpc_client.call("getGroundSpeedSensorData", car_name)
        )
        time_stamp = np.uint64(time.time_ns())
        return (
            np.array(
                [
                    self._data[car_name][gss_name].linear_velocity.x_val,
                    self._data[car_name][gss_name].linear_velocity.y_val,
                ]
            ),
            time_stamp,
        )

    def get_gps_data(
        self, gps_name: str, car_name: Optional[str] = None
    ) -> tuple[GpsData, np.uint64]:
        """Get GPS data as (lat, lon)"""
        if car_name is None:
            car_name = self.default_car_name

        assert (
            gps_name in self._data[car_name].keys()
            and self._data_types[car_name][gps_name] == GpsData
        ), f"GPS {gps_name} not found"

        self._data[car_name][gps_name] = GpsData.from_msgpack(
            self.rpc_client.call("getGpsData", gps_name, car_name)
        )
        time_stamp = np.uint64(time.time_ns())
        return (
            self._data[car_name][gps_name],
            time_stamp,
        )

    def get_wheel_speeds(
        self, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        """Get wheel speeds as (omega_fl, omega_fr, omega_rl, omega_rr) (in rad/s)"""
        if car_name is None:
            car_name = self.default_car_name

        self._data[car_name]["wheel_speeds"] = WheelStates.from_msgpack(
            self.rpc_client.call("simGetWheelStates", car_name)
        )
        time_stamp = np.uint64(time.time_ns())
        return (
            np.pi
            / 30
            * np.array(
                [
                    self._data[car_name]["wheel_speeds"].fl_rpm,
                    self._data[car_name]["wheel_speeds"].fr_rpm,
                    self._data[car_name]["wheel_speeds"].rl_rpm,
                    self._data[car_name]["wheel_speeds"].rr_rpm,
                ]
            ),
            time_stamp,
        )
