#  Copyright (c) 2022. Mattéo Berthet EPFL Racing Team Driverless GitHub@MattBrth
import time
from typing import Callable, Optional

import numpy as np

from .client import FSDSClient
from .types import *


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


class Simulation:
    def __init__(
        self,
        ip: str = "127.0.0.1",
        max_steering: float = 30.0,
        camera_name: str = "examplecam",
    ):
        self.max_steering = max_steering
        self.camera_name = camera_name

        self.client = FSDSClient(ip, 41451, 3)

        crash_guard(self.client.confirmConnection, "confirmConnection")

        crash_guard(self.client.restart, "restart")

        crash_guard(
            lambda: self.client.enableApiControl(True),
            "enableAPIControl",
            self.client.restart,
        )

        self.image = None

        self.state = None

        self.update_state()

    def update_state(self):
        """Get the current state of the vehicle"""
        self.state = self.client.simGetGroundTruthKinematics()

    def calculate_throttle_brake(self, throttle: float):
        """
        Calculate the throttle and brake values from the throttle input.
        """
        if throttle > 0:
            return throttle, 0
        else:
            return 0, throttle

    def calculate_steering(self, steer: float) -> float:
        """Steer is in radians and in [-max_steering, max_steering]"""
        control_steering = -steer / np.deg2rad(self.max_steering)
        return control_steering

    def yaw_rate(self) -> float:
        """
        Yaw rate of the vehicle in radians per second (anti-clockwise).
        """
        return self.state.angular_velocity.z_val

    def yaw(self) -> float:
        """
        Yaw of the vehicle in radians.
        Convention: The 0 rads line is the X axis and the yaw is measured anti-clockwise.
        """
        qx = self.state.orientation.x_val
        qy = self.state.orientation.y_val
        qz = self.state.orientation.z_val
        qw = self.state.orientation.w_val

        yaw = (
            np.mod(
                np.arctan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
                + np.pi / 2
                + np.pi,
                2 * np.pi,
            )
            - np.pi
        )
        return yaw

    def speed(self) -> float:
        """
        Speed of the vehicle in the referential of the track.
        """
        return np.hypot(
            self.state.linear_velocity.x_val, self.state.linear_velocity.y_val
        )

    def speed_longitudinal(self) -> float:
        """
        Speed of the vehicle in the direction of the car.
        """
        angle_vx_vy = np.arctan2(
            self.state.linear_velocity.x_val, -self.state.linear_velocity.y_val
        )
        return self.speed() * np.cos(angle_vx_vy - self.yaw())

    def available(self) -> bool:
        return self.client.ping()

    def get_states(self) -> np.ndarray:
        """
        Get the current state of the vehicle.
        """
        x = self.x()
        y = self.y()
        yaw = self.yaw()
        yaw_rate = self.yaw_rate()
        speed = self.speed_longitudinal()

        return np.array([x, y, yaw, yaw_rate, speed])

    def update_image(self):
        """
        Get the image string from the simulation.
        """
        [self.image] = self.client.simGetImages(
            [
                ImageRequest(
                    camera_name=self.camera_name,
                    image_type=ImageType.Scene,
                    pixels_as_float=False,
                    compress=False,
                )
            ]
        )

    def get_image(self) -> np.ndarray:
        """
        Get the current image from the camera.
        :return: The image as a numpy array of shape (height, width, 3).
        """
        img1d = np.fromstring(self.image.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape((self.image.height, self.image.width, 3))

        return img_rgb

    def x(self) -> float:
        """
        First global cartesian coordinate.
        Convention: At the beginning of the simulation the X-axis is oriented from left to right of the car.
        """
        return -self.state.position.y_val

    def y(self) -> float:
        """
        Second global cartesian coordinate.
        Convention: At the beginning of the simulation the Y-axis is oriented in the same as the car.
        """
        return self.state.position.x_val

    def send_control(self, steering: float, throttle: float):
        """
        Send control to the vehicle.
        Steering is in [-max_steering, max_steering], throttle is in [-1, 1]
        """
        car_controls = CarControls()
        car_controls.steering = self.calculate_steering(steering)
        car_controls.throttle, car_controls.brake = self.calculate_throttle_brake(
            throttle
        )
        self.client.setCarControls(car_controls)


def sleep_sub_ms(delay):
    """Function to provide accurate time delay in seconds"""
    end = time.perf_counter() + delay
    while time.perf_counter() < end:
        pass
