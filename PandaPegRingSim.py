from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from PandaRobot import *
import math

import matplotlib.pyplot as plt
import numpy as np
import scipy 

class PickAndPlaceFSM:
    def __init__(self):
        self.state = "Idle"

    def on_event(self, event):
        # Usiamo elif per assicurarci di processare un solo cambio di stato per chiamata
        if self.state == "Idle":
            if event == "approach":
                self.state = "Approach"
                print("Moving to approach position...")

        elif self.state == "Approach":
            if event == "reached":
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
    # alpha = 1 
    return[
        start[0]+alpha*(end[0]-start[0]),
        start[1]+alpha*(end[1]-start[1]),
        start[2]+alpha*(end[2]-start[2])
    ] 



client = RemoteAPIClient()
sim = client.require('sim')
simIK = client.require('simIK')
panda = PandaRobot(client, "Franka")

#Ring handles
red_ring = sim.getObject(':/Red_ring')
yellow_ring = sim.getObject(':/Yellow_ring')
blue_ring = sim.getObject(':/Blue_ring')
#green_ring= sim.getObject(':/Green_Ring')
# Peg Handle
red_peg = sim.getObject(':/base_peg/Red_peg')
yellow_peg = sim.getObject(':/Yellow_peg')
blue_peg = sim.getObject(':/Blue_peg')
#green_peg = sim.getObject(':/Green_peg')
target = sim.getObject(':/Franka/Target')
# 1. Recupera gli handle dei giunti delle dita prima del ciclo while
finger1 = sim.getObject(':/panda_finger_joint1')
finger2 = sim.getObject(':/panda_finger_joint2')


target_position = sim.getObjectPosition(target, sim.handle_world)
position_red_ring = sim.getObjectPosition(red_ring, sim.handle_world)
position_yellow_ring = sim.getObjectPosition(yellow_ring, sim.handle_world)
position_blue_ring = sim.getObjectPosition(blue_ring, sim.handle_world)
#position_green_ring = sim.getObjectPosition(green_ring, sim.handle_world)

position_red_peg = sim.getObjectPosition(red_peg, sim.handle_world)
position_yellow_peg = sim.getObjectPosition(yellow_peg, sim.handle_world)
position_blue_peg = sim.getObjectPosition(blue_peg, sim.handle_world)
#position_green_peg = sim.getObjectPosition(green_peg, sim.handle_world)


#Adjust the height of the peg
position_red_peg[2] += 0.1
position_blue_peg[2] += 0.1
position_yellow_peg[2] += 0.1
#position_green_peg[2] += 0.1


start_time = 0.0
duration = 50.0
end_time = start_time + duration


fsm = PickAndPlaceFSM()
panda.startSimulation()
alpha = 1 
fsm.on_event("approach")  # Start the FSM

panda.startSimulation()

new_red = position_red_ring
new_red[2] += 0.1
new_red[1] += 0.05

while (t := panda.simulationTime()) < 50:
    stato = fsm.current_state()
    alpha = (t - start_time) / duration  # Calculate alpha based on elapsed time
    print(f"Current FSM State: {stato} at time {t:.2f}s")
    
    if stato == "Approach":
        # 1. Apri il gripper
        sim.setJointTargetPosition(finger1, 0.04)
        sim.setJointTargetPosition(finger2, 0.04)

        # 2. Muovi il target verso l'anello rosso (qui useresti il lerp nel tempo)
        
        
        print("New target position for approach:", new_red)
        nnew_red = lerp_3d(new_red, position_red_ring, alpha)
        sim.setObjectPosition(target, nnew_red , sim.handle_world)
        fsm.on_event("reached")  # Transition to Pick state

    if stato == "Pick":
            print("stato")
            # red = lerp_3d(new_red, position_red_ring, alpha)
            # sim.setObjectPosition(target, red , sim.handle_world)
            sim.setJointTargetPosition(finger1, 0.0)
            sim.setJointTargetPosition(finger2, 0.0)

    
    panda.stepSimulation()         
               
    
    panda.stepSimulation()

panda.stopSimulation()