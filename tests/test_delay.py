import numpy as np

from fsds_client import LowLevelClient
from time import sleep
import time
import os

HOST = "127.0.0.1"

# from platform import uname
# if 'linux' in uname().system.lower() and 'microsoft' in uname().release.lower(): # In WSL2
#     if 'WSL_HOST_IP' in os.environ:
#         HOST = os.environ['WSL_HOST_IP']
#
# print("Using WSL2 Host IP address: ", HOST)

simulation = LowLevelClient(ip=HOST)
sleep(2)
simulation.client.enableApiControl(False),
moy = np.array([])
for i in range(100):
    start = time.time()
    simulation.update_image()
    simulation.get_image()
    moy = np.append(moy, time.time() - start)
print("Get image delay :", round(np.mean(moy) * 1000, 1), "ms")
moy = np.array([])
for i in range(100):
    start = time.time()
    simulation.update_state()
    simulation.get_states()
    moy = np.append(moy, time.time() - start)
print("Get state delay :", round(np.mean(moy) * 1000, 1), "ms")
