#  Copyright (c) 2022. Mattéo Berthet EPFL Racing Team Driverless GitHub@MattBrth
import time
from typing import Callable, Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from .low_level_client import LowLevelClient
from .types import *
from .utils import *
import track_database as tdb

__all__ = ["HighLevelClient"]


def crash_guard(callback: Callable, error_message: str, aux: Optional[Callable] = None):
    """
    Use this function to automatically restart the simulation if it crashes.

    :param callback:
    :param error_message:
    :param aux:
    """
    try:
        callback()
    except:
        print("FAILED : ", error_message)
        simulation_crashed = True
        while simulation_crashed:
            time.sleep(1.0)
            try:
                if aux is not None:
                    aux()

                callback()
                simulation_crashed = False
            except:
                print("FAILED : ", error_message)
                simulation_crashed = True


class HighLevelClient:
    _delta_max: float = np.deg2rad(35.0)
    camera_name: str
    low_level_client: LowLevelClient
    track: tdb.Track
    cones_positions: np.ndarray
    state: KinematicsState
    image: Optional[ImageResponse]

    def __init__(
        self,
        ip: str = "127.0.0.1",
        camera_name: str = "examplecam",
    ):
        self.camera_name = camera_name
        self.low_level_client = LowLevelClient(ip, 41451, 3)
        crash_guard(self.low_level_client.confirmConnection, "confirmConnection")
        crash_guard(self.low_level_client.restart, "restart")
        sleep(1)
        crash_guard(
            lambda: self.low_level_client.enableApiControl(True),
            "enableAPIControl",
            self.low_level_client.restart,
        )
        self.image = None
        map_name = self.get_map_name().removesuffix("_cones")
        assert map_name in tdb.available_tracks, f"Map {map_name} not available"
        self.track = tdb.load_track(map_name)
        self.state = self.low_level_client.simGetGroundTruthKinematics()
        self.cones_positions = np.vstack(
            (self.track.right_cones, self.track.left_cones)
        )

    # utils ==================================================================

    def set_api_control(self, flag: bool):
        """
        Enable or disable API control.
        :param flag: True to enable, False to disable.
        """
        self.low_level_client.enableApiControl(flag)

    def available(self) -> bool:
        return self.low_level_client.ping()

    def update_state(self):
        """Get the current state of the vehicle"""
        self.state = self.low_level_client.simGetGroundTruthKinematics()

    # interact with the simulation ===========================================
    def get_state(self) -> np.ndarray:
        """Get the current state of the vehicle in the following order: [X,Y,phi,v_x,v_y,r]"""
        self.state = self.low_level_client.simGetGroundTruthKinematics()
        # position
        x = self.state.position.x_val
        y = self.state.position.y_val
        # orientation
        phi = (
            np.mod(to_eularian_angles(self.state.orientation)[2] + np.pi, 2 * np.pi)
            - np.pi
        )
        # linear velocity
        v = np.hypot(self.state.linear_velocity.x_val, self.state.linear_velocity.y_val)
        angle_vx_vy = np.arctan2(
            self.state.linear_velocity.y_val, self.state.linear_velocity.x_val
        )
        v_x = v * np.cos(angle_vx_vy - phi)
        v_y = v * np.sin(angle_vx_vy - phi)
        # angular velocity
        r = self.state.angular_velocity.z_val

        return np.array([x, y, phi, v_x, v_y, r])

    def send_control(self, T: float, delta: float):
        """
        :param T: Throttle value between -1 and 1.
        :param delta: Steering angle in radians between -max_steering and max_steering.
        """
        car_controls = CarControls()
        car_controls.steering = (
            -delta / self._delta_max
        )  # delta is anti-clockwise but steering is clockwise
        if T > 0:
            car_controls.throttle, car_controls.brake = T, 0
        else:
            car_controls.throttle, car_controls.brake = 0, T

        self.low_level_client.setCarControls(car_controls)

    def find_cones_lidar(
        self, max_cone_distance: float = 20.0, max_group_dispersion: float = 0.2
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        Simple lidar cone detection algorithm that returns the positions of the cones in
        a local cartesian frame, i.e. X is forward, Y is left.
        :returns: cones_positions and raw 2D point cloud, all in local cartesian coordinates.
        """
        # Get the pointcloud
        lidardata = self.low_level_client.getLidarData()
        if len(lidardata.point_cloud) < 3:
            # not enough points
            return np.array([]), np.array([])

        # Convert the list of floats into a list of local XYZ coordinates (relative to
        # the lidar pose)
        points = np.array(lidardata.point_cloud, dtype=np.dtype("f4"))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))

        # discard the points that are too low, too high or too far away
        roll_pitch_yaw = to_eularian_angles(lidardata.pose.orientation)
        lidar_orientation = R.from_euler("ZYX", roll_pitch_yaw[::-1], degrees=False)
        altitudes = (
            lidar_orientation.inv().apply(points)[:, 2] + lidardata.pose.position.z_val
        )
        points = points[
            (altitudes > 0.2)
            & (altitudes < 0.5)
            & (np.linalg.norm(points, axis=1) < max_cone_distance)
            & (np.linalg.norm(points, axis=1) > 2.0)
        ]
        points = points[:, :2]

        # Go through all the points and find nearby groups of points that are close
        # together as those will probably be cones.
        distances = np.sqrt(np.sum(np.square(np.diff(points, axis=0)), axis=1))
        groups = np.split(points, np.where(distances > max_group_dispersion)[0] + 1)
        cones = []
        for group in groups:
            if np.max(np.abs(np.std(group, axis=0))) <= max_group_dispersion / np.sqrt(
                len(group)
            ):
                cones.append(np.mean(group, axis=0))

        cones = np.array(cones)

        return cones, points

    def find_cones(
        self,
        state: np.ndarray,
        noise_cov: np.ndarray = np.diag([0.0, 0.0]),
        max_distance: float = 20.0,
        coords_type: str = "polar",
    ) -> np.ndarray:
        assert coords_type in [
            "polar",
            "cartesian",
        ], "coords_type must be polar or cartesian"
        cartesian = (self.cones_positions - state[:2]) @ np.array(
            [
                [np.cos(-state[2]), -np.sin(-state[2])],
                [np.sin(-state[2]), np.cos(-state[2])],
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
            (-np.pi / 2 <= polar[:, 1])
            & (polar[:, 1] <= np.pi / 2)
            & (polar[:, 0] <= max_distance),
            :,
        ]

        # add noise to the polar coordinates
        polar += np.random.multivariate_normal(
            np.zeros(2), noise_cov, size=polar.shape[0]
        )
        if coords_type == "polar":
            return polar
        else:
            cartesian = np.hstack(
                (
                    np.expand_dims(polar[:, 0] * np.cos(polar[:, 1]), 1),
                    np.expand_dims(polar[:, 0] * np.sin(polar[:, 1]), 1),
                )
            )
            return cartesian

    def get_image(self) -> np.ndarray:
        """
        Get the current image from the camera.
        :return: The image as a numpy array of shape (height, width, 3).
        """
        [self.image] = self.low_level_client.simGetImages(
            [
                ImageRequest(
                    camera_name=self.camera_name,
                    image_type=ImageType.Scene,
                    pixels_as_float=False,
                    compress=False,
                )
            ]
        )
        img1d = np.fromstring(self.image.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape((self.image.height, self.image.width, 3))

        return img_rgb

    def get_wheels_speed(self):
        front_left = self.low_level_client.simGetWheelStates().fl_rpm
        front_right = self.low_level_client.simGetWheelStates().fr_rpm
        rear_left = self.low_level_client.simGetWheelStates().rl_rpm
        rear_right = self.low_level_client.simGetWheelStates().rr_rpm

        return np.array([front_left, front_right, rear_left, rear_right])

    def get_map_name(self) -> str:
        return self.low_level_client.getMapName()
