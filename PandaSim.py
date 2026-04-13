from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PandaRobot import *

import matplotlib.pyplot as plt
import numpy as np

# Python examples 
# https://github.com/CoppeliaRobotics/zmqRemoteApi/tree/master/clients/python 

client = RemoteAPIClient()

panda = PandaRobot(client, "Panda")

panda.startSimulation()

while (t := panda.simulationTime()) < 5:
    print(f'Simulation time: {t:.2f} [s]')

    panda.getStatus()

    panda.stepSimulation()

panda.stopSimulation()