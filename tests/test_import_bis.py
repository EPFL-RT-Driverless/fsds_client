import numpy as np

import fsds_client
from time import sleep
import time
import os

# HOST = '127.0.0.1' # Standard loopback interface address (localhost)
# from platform import uname
# if 'linux' in uname().system.lower() and 'microsoft' in uname().release.lower(): # In WSL2
#     if 'WSL_HOST_IP' in os.environ:
#         HOST = os.environ['WSL_HOST_IP']
#
# print("Using WSL2 Host IP address: ", HOST)

# simulation = Simulation(ip=HOST)
simulation = fsds_client.Simulation(ip="127.0.0.1")
sleep(2)
simulation.client.enableApiControl(True),
moy = np.array([])
for i in range(1000):
    start = time.time()
    simulation.send_control(1, 1)
    moy = np.append(moy, time.time() - start)
print("Get image delay :", round(np.mean(moy) * 1000, 1), "ms")


simulation.client.simPause(True)
fsds_client.sleep_sub_ms(0.01)
simulation.client.simPause(False)

sleep(20)
