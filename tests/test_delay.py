import time
from time import sleep

import numpy as np

from fsds_client import HighLevelClient
from fsds_client.utils import sleep_sub_ms

HOST = "127.0.0.1"

# from platform import uname
# if 'linux' in uname().system.lower() and 'microsoft' in uname().release.lower(): # In WSL2
#     if 'WSL_HOST_IP' in os.environ:
#         HOST = os.environ['WSL_HOST_IP']
#
# print("Using WSL2 Host IP address: ", HOST)

simulation = HighLevelClient(ip=HOST)
sleep(2)
simulation.client.enableApiControl(False),
moy = np.array([])

print(simulation.client.getMapName())

for i in range(100):
    start = time.time()
    simulation.get_image()
    moy = np.append(moy, time.time() - start)
print("Get image delay :", round(np.mean(moy) * 1000, 1), "ms")
moy = np.array([])
for i in range(100):
    start = time.time()
    simulation.get_state()
    moy = np.append(moy, time.time() - start)
print("Get state delay :", round(np.mean(moy) * 1000, 1), "ms")

moy = np.array([])
for i in range(100):
    start = time.time()
    simulation.find_cones()
    moy = np.append(moy, time.time() - start)
print("Get lidar delay :", round(np.mean(moy) * 1000, 1), "ms")

simulation.client.simPause(True)
sleep_sub_ms(0.01)
simulation.client.simPause(False)

sleep(20)
