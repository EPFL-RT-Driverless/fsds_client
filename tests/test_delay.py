import numpy as np

from fsds_client import *
from time import time_ns, perf_counter
from fsds_client.utils import sleep
import tqdm


def bruh(call):
    start = time_ns()
    res = call()
    end = time_ns()
    if isinstance(res, tuple):
        timestamp = res[1]
        data = res[0]
        return print_bruh(data, end, start, timestamp)
    if isinstance(res, list):
        final_res = []
        for data, timestamp in res:
            final_res.append(print_bruh(data, end, start, timestamp))
        return final_res


def print_bruh(data, end, start, timestamp):
    print(
        f"Call took {(end - start) / 1e6} ms, difference from timestamp: {(end - timestamp) / 1e6}ms (i.e. timestamp is {(timestamp - start) / 1e6} ms after call)"
    )
    return data, timestamp


def main():
    client = FSDSClient()
    # print("State: ", end="")
    # state, timestamp = bruh(client.get_state)
    # print("1 image: ", end="")
    # [(image, timestamp)] = bruh(
    #     lambda: client.get_image(camera_names=["camera1"], compressed=False)
    # )
    # print("2 images: ", end="")
    # [(image, timestamp)] = bruh(lambda: client.get_image(camera_names=["camera1"]))
    # print("3 images: ", end="")
    # [(image, timestamp), (image2, timestamp2)] = bruh(
    #     lambda: client.get_image(camera_names=["camera1", "camera2"])
    # )
    # print("point cloud: ", end="")
    # pointcloud, timestamp = bruh(lambda: client.get_point_cloud("lidar1"))
    # print("imu: ", end="")
    # imu_data, timestamp = bruh(lambda: client.get_imu_data("imu"))
    # print("gss: ", end="")
    # gss_data, timestamp = bruh(lambda: client.get_gss_data("gss"))
    # print("gps: ", end="")
    # gps_data, timestamp = bruh(lambda: client.get_gps_data("gps"))
    # print("wheel speeds: ", end="")
    # wheel_speeds, timestamp = bruh(client.get_wheel_speeds)

    runtimes = []
    images = []
    for i in tqdm.tqdm(range(100)):
        start = perf_counter()
        [(image, timestamp)] = client.get_image(
            camera_names=["camera1"], compressed=True
        )
        stop = perf_counter()
        runtimes.append(1000 * (stop - start))
        images.append(image)
    runtimes = np.array(runtimes)
    print(f"Average runtime: {np.mean(runtimes)} ms")
    print(f"Std runtime: {np.std(runtimes)} ms")


if __name__ == "__main__":
    main()
