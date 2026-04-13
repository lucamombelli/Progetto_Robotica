from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PandaRobot import *

import matplotlib.pyplot as plt
import numpy as np
import scipy 

class PickAndPlaceFSM:
    def __init__(self):
        self.state = "Idle"

    def on_event(self, event):
        if self.state == "Idle":
            if event == "start":
                self.state = "Pick"
                print("Starting operation: Picking object.")

        elif self.state == "Pick":
            if event == "picked":
                self.state = "Move"
                print("Object picked: Moving to place position.")

        elif self.state == "Move":
            if event == "arrived":
                self.state = "Place"
                print("Arrived at place location: Placing object.")

        elif self.state == "Place":
            if event == "placed":
                self.state = "Return"
                print("Object placed: Returning to start.")

        elif self.state == "Return":
            if event == "returned":
                self.state = "Idle"
                print("Returned to start: Ready for next task.")

    def current_state(self):
        return self.state



def lerp_3d(start, end, alpha):
    alpha = max(0.0, min(1.0, alpha))  # Clamp alpha to [0, 1]  
    alpha = 1 
    return[
        start[0]+alpha*(end[0]-start[0]),
        start[1]+alpha*(end[1]-start[1]),
        start[2]+alpha*(end[2]-start[2])
    ] 



client = RemoteAPIClient()
sim = client.require('sim')
simIK = client.require('simIK')
panda = PandaRobot(client, "Franka")


red_ring = sim.getObject(':/Red_ring')
yellow_ring = sim.getObject(':/Yellow_ring')
blue_ring = sim.getObject(':/Blue_ring')
red_peg = sim.getObject(':/base_peg/Red_peg')
yellow_peg = sim.getObject(':/Yellow_peg')
blue_peg = sim.getObject(':/Blue_peg')
target = sim.getObject(':/Franka/Target')
# 1. Recupera gli handle dei giunti delle dita prima del ciclo while
finger1 = sim.getObject(':/panda_finger_joint1')
finger2 = sim.getObject(':/panda_finger_joint2')


target_position = sim.getObjectPosition(target, sim.handle_world)
position_red_ring = sim.getObjectPosition(red_ring, sim.handle_world)
position_yellow_ring = sim.getObjectPosition(yellow_ring, sim.handle_world)
position_blue_ring = sim.getObjectPosition(blue_ring, sim.handle_world)

position_red_peg = sim.getObjectPosition(red_peg, sim.handle_world)
position_yellow_peg = sim.getObjectPosition(yellow_peg, sim.handle_world)
position_blue_peg = sim.getObjectPosition(blue_peg, sim.handle_world)


#Adjust the height of the peg
position_red_peg[2] += 0.1
position_blue_peg[2] += 0.1
position_yellow_peg[2] += 0.1


start_time = 0.0
duration = 30.0
end_time = start_time + duration


fsm = PickAndPlaceFSM()
panda.startSimulation()

alpha = 1 

fsm = PickAndPlaceFSM()

fsm.on_event("start")  # Start the FSM

panda.startSimulation()

while (t := panda.simulationTime()) < 20:
    stato = fsm.current_state()
    if stato == "Pick":
        # 1. Apri il gripper
        sim.setJointTargetPosition(finger1, 0.04)
        sim.setJointTargetPosition(finger2, 0.04)

        # 2. Muovi il target verso l'anello rosso (qui useresti il lerp nel tempo)
        sim.setObjectPosition(target ,  position_red_ring ,sim.handle_world)

        if (result :=sim.checkCollision(target, red_ring, 1)) == True :
            fsm.on_event("picked")  # Transition to Move state
            sim.setJointTargetPosition(finger1, 0.0)
            sim.setJointTargetPosition(finger2, 0.0)   
            alpha = (t - start_time) / duration  # Calculate alpha based on elapsed time
            red = lerp_3d(position_red_ring, position_red_peg, alpha)
            sim.setObjectPosition(target, sim.handle_world, red)
        else :
            print("Not in collision yet, keep trying...")
    
    panda.stepSimulation()

panda.stopSimulation()