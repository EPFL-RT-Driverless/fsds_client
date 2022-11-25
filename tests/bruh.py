import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter
from fsds_client import HighLevelClient
from fsds_client.utils import sleep_sub_ms


def main():
    sim = HighLevelClient()
    sleep_sub_ms(0.1)
    # sim.client.enableApiControl(False)
    # y = [sim.get_state()[1]]
    y = []
    sim.send_control(1.0, 0.0)
    a_x = []
    a_y = []
    while True:
        # plt.axis("equal")
        # plt.clf()
        # state = sim.get_state()
        # print("State:", state)
        # plt.scatter(state[0], state[1], color="red", marker="x")
        # start = perf_counter()
        # cones, points = sim.find_cones_lidar()
        # stop = perf_counter()
        # print("Time to find cones: {} ms".format(1000 * (stop - start)))
        # rot = np.array(
        #     [
        #         [np.cos(state[2]), -np.sin(state[2])],
        #         [np.sin(state[2]), np.cos(state[2])],
        #     ]
        # )
        # if cones.size > 0 and points.size > 0:
        #     cones = cones @ rot.T
        #     cones += state[:2]
        #     points = points @ rot.T
        #     points += state[:2]
        #     plt.scatter(cones[:, 0], cones[:, 1], color="blue", marker="^", zorder=1)
        #     plt.scatter(
        #         points[:, 0],
        #         points[:, 1],
        #         color="green",
        #         marker="o",
        #         s=1,
        #         zorder=2,
        #     )
        # plt.axis("equal")
        # plt.pause(0.5)
        start = perf_counter()
        state = sim.get_state()
        y.append(state[1])
        imu_data = sim.client.getImuData()
        a_x.append(imu_data.linear_acceleration.x_val)
        a_y.append(imu_data.linear_acceleration.y_val)
        if state[3] >= 20.0:
            break
        stop = perf_counter()
        sleep_sub_ms(0.1 - (stop - start))

    y = np.array(y)
    a_x = np.array(a_x)
    a_y = np.array(a_y)
    N = len(y)
    print(a_x)
    # plt.subplots(121)
    plt.plot(np.arange(N) * 0.1, y)
    # plt.title(122)
    plt.plot(0.1 * np.arange(N), a_x)
    plt.plot(0.1 * np.arange(N), a_y)
    plt.show()


if __name__ == "__main__":
    main()
