import numpy as np
import matplotlib.pyplot as plt
from time import perf_counter
from fsds_client import HighLevelClient
from fsds_client.utils import sleep_sub_ms


def main():
    coords_type = "cartesian"
    client = HighLevelClient()
    client.low_level_client.enableApiControl(False)
    sleep_sub_ms(0.1)
    plt.figure()
    while True:
        plt.clf()
        plt.scatter(
            client.cones_positions[:, 0],
            client.cones_positions[:, 1],
            c="r",
            s=3,
            marker="^",
        )
        state = client.get_state()
        plt.scatter(state[0], state[1], color="red", marker="x")
        start = perf_counter()
        cones = client.find_cones(
            state=state, noise_cov=np.diag([0.1, 0.0]), coords_type=coords_type
        )
        stop = perf_counter()
        print("Time to find cones: {} ms".format(1000 * (stop - start)))
        if coords_type == "polar":
            plt.scatter(
                state[0] + cones[:, 0] * np.cos(cones[:, 1] + state[2]),
                state[1] + cones[:, 0] * np.sin(cones[:, 1] + state[2]),
                color="blue",
                marker="^",
                s=10,
                zorder=2,
            )
        else:
            cones = (
                cones
                @ np.array(
                    [
                        [np.cos(state[2]), -np.sin(state[2])],
                        [np.sin(state[2]), np.cos(state[2])],
                    ]
                ).T
                + state[:2]
            )
            plt.scatter(
                cones[:, 0], cones[:, 1], color="blue", marker="^", s=10, zorder=2
            )

        # rot = np.array(
        #     [
        #         [np.cos(state[2]), -np.sin(state[2])],
        #         [np.sin(state[2]), np.cos(state[2])],
        #     ]
        # )
        # if cones.size > 0:
        #     cones = cones @ rot.T
        #     cones += state[:2]
        #     plt.scatter(
        #         cones[:, 0], cones[:, 1], s=10, color="blue", marker="^", zorder=3
        #     )

        plt.axis("equal")
        plt.pause(0.5)


if __name__ == "__main__":
    main()
