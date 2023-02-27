import json
import os
import warnings
from functools import cached_property
from typing import Optional, Callable, Union, Type

import msgpackrpc
import numpy as np

import track_database as tdb
from .types import *
from .utils import *

__all__ = ["FSDSClient"]


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
        default_car_name="FSDSCar",
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

            # self._data = {
            #     car_name: {
            #         sensor_name: None
            #         for sensor_name in settings["Vehicles"][car_name]["Sensors"].keys()
            #         if settings["Vehicles"][car_name]["Sensors"][sensor_name]["Enabled"]
            #     }
            #     | {
            #         camera_name: ImageResponse()
            #         for camera_name in settings["Vehicles"][car_name]["Cameras"].keys()
            #     }
            #     | {
            #         "state": KinematicsState(),
            #         "control": CarControls(),
            #         "yaw": 0.0,
            #         "cones_observations": np.empty((0, 2)),
            #     }
            #     for car_name in settings["Vehicles"].keys()
            # }

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

        self.set_api_control(api_control)
        map_name = self.map_name
        assert (
            map_name in tdb.available_tracks
        ), f"Map {map_name} not found in track database"
        self.track = tdb.load_track(map_name)
        self._all_cones = np.vstack((self.track.right_cones, self.track.left_cones))

    @property
    def car_names(self):
        return list(self._data.keys())

    def camera_names(self, car_name: Optional[str] = None):
        if car_name is None:
            car_name = self.default_car_name
        return filter(
            lambda k: self._data_types[car_name][k] == ImageResponse,
            self._data[car_name].keys(),
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

    def ping(self) -> bool:
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

    def get_api_control(self, car_name: Optional[str] = None) -> bool:
        if car_name is None:
            car_name = self.default_car_name
        return self._try_and_fail(
            lambda: self.rpc_client.call("isApiControlEnabled", car_name),
            "Failed to get API control",
        )

    def set_api_control(self, enabled: bool, car_name: Optional[str] = None):
        if car_name is None:
            car_name = self.default_car_name
        self._try_and_fail(
            lambda: self.rpc_client.call("enableApiControl", enabled, car_name),
            "Failed to set API control",
        )

    def get_state(self, car_name: Optional[str] = None) -> tuple[np.ndarray, np.uint64]:
        if car_name is None:
            car_name = self.default_car_name

        def bruh() -> tuple[np.ndarray, np.uint64]:
            self._data[car_name]["state"] = KinematicsState.from_msgpack(
                self.rpc_client.call("simGetGroundTruthKinematics", car_name)
            )
            state = self._data[car_name]["state"]

            # position
            x = state.position.x_val
            y = state.position.y_val
            # orientation
            phi = (
                np.mod(to_eularian_angles(state.orientation)[2] + np.pi, 2 * np.pi)
                - np.pi
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
                state.time_stamp,
            )

        return self._try_and_fail(bruh, "Failed to get state")

    def set_control(self, control: np.ndarray, car_name: Optional[str] = None):
        if car_name is None:
            car_name = self.default_car_name

        assert control.shape == (2,)
        np.clip(control[0], -1, 1, out=control[0])
        np.clip(control[1], -self.delta_max, self.delta_max, out=control[1])

        self._data[car_name]["control"] = CarControls()
        (
            self._data[car_name]["control"].throttle,
            self._data[car_name]["control"].brake,
        ) = (
            (control[0], 0.0) if control[0] > 0 else (0.0, control[0])
        )
        self._data[car_name]["control"].steering = -control[1] / self.delta_max

        self._try_and_fail(
            lambda: self.rpc_client.call(
                "simSetVehicleControls", self.car_name, self._data[car_name]["control"]
            ),
            "Failed to set control",
        )

    def get_cones_observations(
        self, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        if car_name is None:
            car_name = self.default_car_name

        # TODO: here do we just use the last state or do we call the API again?
        # self.get_state(car_name)
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
        self, camera_names: list[str] = [], car_name: Optional[str] = None
    ) -> list[np.ndarray, np.uint64]:
        if car_name is None:
            car_name = self.default_car_name

        assert len(camera_names) > 0
        for camera_name in camera_names:
            assert (
                camera_name in self._data[car_name]
                and self._data_types[car_name][camera_name] == ImageResponse
            ), f"Camera {camera_name} not found"

        def bruh():
            responses = [
                ImageResponse.from_msgpack(response)
                for response in self.rpc_client.call(
                    "simGetImages",
                    [
                        ImageRequest(
                            camera_name=camera_name,
                            image_type=ImageType.Scene,
                            compress=False,
                        )
                        for camera_name in camera_names
                    ],
                    car_name,
                )
            ]
            for i, camera_name in enumerate(camera_names):
                self._data[car_name][camera_name] = responses[i]

            # TODO: should we use fromstring or frombuffer?
            return [
                (
                    np.fromstring(response.image_data_uint8, dtype=np.uint8).reshape(
                        (response.height, response.width, 3)
                    ),
                    response.timestamp,
                )
                for response in responses
            ]

        return self._try_and_fail(
            bruh,
            "Failed to get image",
        )

    def get_point_cloud(
        self, lidar_name: str, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        if car_name is None:
            car_name = self.default_car_name
        assert (
            lidar_name in self._data[car_name].keys()
            and self._data_types[car_name][lidar_name] == LidarData
        ), f"LiDAR {lidar_name} not found"

        def bruh():
            self._data[car_name][lidar_name] = LidarData.from_msgpack(
                self.rpc_client.call("getLidarData", lidar_name, car_name)
            )
            return (
                self._data[car_name][lidar_name].point_cloud,
                self._data[car_name][lidar_name].time_stamp,
            )

        return self._try_and_fail(
            bruh,
            "Failed to get point cloud",
        )

    def get_imu_data(
        self, imu_name: str, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        """Get IMU data as (phi, ax, ay, r)"""
        if car_name is None:
            car_name = self.default_car_name

        assert (
            "imu_data" in self._data[car_name].keys()
            and self._data_types[car_name]["imu_data"] == ImuData
        ), f"IMU {imu_name} not found"

        def bruh():
            self._data[car_name]["imu_data"] = ImuData.from_msgpack(
                self.rpc_client.call("getImuData", car_name)
            )
            return (
                np.array(
                    [
                        np.mod(
                            to_eularian_angles(
                                self._data[car_name]["imu_data"].orientation
                            )[2]
                            + np.pi,
                            2 * np.pi,
                        )
                        - np.pi,
                        self._data[car_name]["imu_data"].linear_acceleration.x_val,
                        self._data[car_name]["imu_data"].linear_acceleration.y_val,
                        self._data[car_name]["imu_data"].angular_velocity.z_val,
                    ]
                ),
                self._data[car_name]["imu_data"].time_stamp,
            )

        return self._try_and_fail(
            bruh,
            f"Failed to get IMU data from {imu_name} on car {car_name}",
        )

    def get_gss_data(self, gss_name: str, car_name: Optional[str] = None):
        """Get GSS data as (vx, vy)"""
        if car_name is None:
            car_name = self.default_car_name

        assert (
            "gss_data" in self._data[car_name].keys()
            and self._data_types[car_name]["gss_data"] == GpsData
        ), f"GSS {gss_name} not found"

        def bruh():
            self._data[car_name]["gss_data"] = GroundSpeedSensorData.from_msgpack(
                self.rpc_client.call("getGpsData", car_name)
            )
            return (
                np.array(
                    [
                        self._data[car_name]["gss_data"].linear_velocity.x_val,
                        self._data[car_name]["gss_data"].linear_velocity.y_val,
                    ]
                ),
                self._data[car_name]["gss_data"].time_stamp,
            )

        return self._try_and_fail(
            bruh,
            "Failed to get GSS data",
        )

    def get_gps_data(self, gps_name: str, car_name: Optional[str] = None) -> GpsData:
        """Get GPS data as (lat, lon)"""
        if car_name is None:
            car_name = self.default_car_name

        assert (
            "gps_data" in self._data[car_name].keys()
            and self._data_types[car_name]["gps_data"] == GpsData
        ), f"GPS {gps_name} not found"

        def bruh():
            self._data[car_name]["gps_data"] = GpsData.from_msgpack(
                self.rpc_client.call("getGpsData", car_name)
            )
            return self._data[car_name]["gps_data"]

        return self._try_and_fail(
            bruh,
            "Failed to get GPS data",
        )

    def get_wheel_speeds(
        self, car_name: Optional[str] = None
    ) -> tuple[np.ndarray, np.uint64]:
        """Get wheel speeds as (omega_fl, omega_fr, omega_rl, omega_rr) (in rad/s)"""
        if car_name is None:
            car_name = self.default_car_name

        def bruh():
            self._data[car_name]["wheel_speeds"] = WheelStates.from_msgpack(
                self.rpc_client.call("simGetWheelStates", car_name)
            )
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
                self._data[car_name]["wheel_speeds"].time_stamp,
            )

        return self._try_and_fail(
            bruh,
            "Failed to get wheel speeds",
        )
