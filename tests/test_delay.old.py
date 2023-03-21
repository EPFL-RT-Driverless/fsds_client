import time
from time import sleep

import numpy as np

from fsds_client import HighLevelClient
from fsds_client.utils import sleep

HOST = "127.0.0.1"

# from platform import uname
# if 'linux' in uname().system.lower() and 'microsoft' in uname().release.lower(): # In WSL2
#     if 'WSL_HOST_IP' in os.environ:
#         HOST = os.environ['WSL_HOST_IP']
#
# print("Using WSL2 Host IP address: ", HOST)

simulation = HighLevelClient(ip=HOST)
sleep(2)
simulation.low_level_client.enableApiControl(False),
moy = np.array([])

print(simulation.low_level_client.getMapName())

for i in range(100):
    start = time.perf_counter()
    simulation.get_image()
    moy = np.append(moy, time.perf_counter() - start)
print("Get image delay :", round(np.mean(moy) * 1000, 1), "ms")
moy = np.array([])
for i in range(100):
    start = time.perf_counter()
    simulation.get_state()
    moy = np.append(moy, time.perf_counter() - start)
print("Get state delay :", round(np.mean(moy) * 1000, 1), "ms")

moy = np.array([])
for i in range(100):
    start = time.perf_counter()
    simulation.find_cones_lidar()
    moy = np.append(moy, time.perf_counter() - start)
print("Get lidar delay :", round(np.mean(moy) * 1000, 1), "ms")

simulation.low_level_client.simPause(True)
sleep(0.01)
simulation.low_level_client.simPause(False)

sleep(20)
