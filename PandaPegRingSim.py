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


def lerp_3d(start: tuple[float, float, float], end: tuple[float, float, float], alpha: float) -> list[float]:
    """
    Esegue un'interpolazione lineare tra due punti 3D.
    """
    alpha = max(0.0, min(1.0, alpha))
    return [
        start[0] + alpha * (end[0] - start[0]),
        start[1] + alpha * (end[1] - start[1]),
        start[2] + alpha * (end[2] - start[2])
    ]


client = RemoteAPIClient()
sim = client.require('sim')
simIK = client.require('simIK')
panda = PandaRobot(client, "Franka")

# Base Handle
base = sim.getObject(':/base_peg')

# Ring and Torus handles
# Red Group
red_ring = sim.getObject(':/Red_ring')
red_torus = sim.getObject(':/Red_ring/Torus1')
# Yellow Group
yellow_ring = sim.getObject(':/Yellow_ring')
yellow_torus = sim.getObject(':/Yellow_ring/Torus1')
# Blue Group
blue_ring = sim.getObject(':/Blue_ring')
blue_torus = sim.getObject(':/Blue_ring/Torus2')

# Peg Handle
red_peg = sim.getObject(':/base_peg/Red_peg')
yellow_peg = sim.getObject(':/Yellow_peg')
blue_peg = sim.getObject(':/Blue_peg')
target = sim.getObject(':/Franka/Target')

# Recupera gli handle dei giunti delle dita
finger1 = sim.getObject(':/panda_finger_joint1')
finger2 = sim.getObject(':/panda_finger_joint2')
hand = sim.getObject(':/Franka/panda_hand_visual')
red_cilinder = sim.getObject(':/Cylinder')
blue_cilinder = sim.getObject(':/Cylinder2')
yellow_cilinder = sim.getObject(':/Cylinder6')

# Get the Position for every object of interest
target_position = sim.getObjectPosition(target, sim.handle_world)
position_red_ring = sim.getObjectPosition(red_ring, sim.handle_world)
position_yellow_ring = sim.getObjectPosition(yellow_ring, sim.handle_world)
position_blue_ring = sim.getObjectPosition(blue_ring, sim.handle_world)

position_red_peg = sim.getObjectPosition(red_peg, sim.handle_world)
position_yellow_peg = sim.getObjectPosition(yellow_peg, sim.handle_world)
position_blue_peg = sim.getObjectPosition(blue_peg, sim.handle_world)

base_pos = sim.getObjectPosition(base, sim.handle_world)
base_pos[2] += 0.3

# Adjust the height of the peg
position_red_peg[2] += 0.1
position_blue_peg[2] += 0.1
position_yellow_peg[2] += 0.1

fsm = PickAndPlaceFSM()
fsm.on_event("approach")

panda.startSimulation()

# Salva le posizioni e gli orientamenti di partenza globali
start_pose = sim.getObjectPose(target, sim.handle_world)
start_pos = start_pose[0:3]       
start_orient = start_pose[3:7]

# Coordinate di Approach
new_red = list(position_red_ring)
new_red[2] += 0.2  

new_blue = list(position_blue_ring)
new_blue[2] += 0.2 

new_yellow = list(position_yellow_ring)
new_yellow[2] += 0.1
new_yellow[1] += 0.01

# Coordinate Pick
pick_target_red = list(position_red_ring)
pick_target_red[2] += 0 
pick_target_red[1] += 0.04  

pick_target_blue = list(position_blue_ring)
pick_target_blue[2] += 0 
pick_target_blue[1] += 0.04  

pick_target_yellow = list(position_yellow_ring)
pick_target_yellow[2] += 0 
pick_target_yellow[1] += 0.04  

# Coordinate Place
place_target_red = list(position_red_peg)
place_target_red[2] += 0.005 
place_target_red[1] -= 0.02  
place_target_red[0] -= 0.07

place_target_blue = list(position_blue_peg)
place_target_blue[2] += 0.05 
place_target_blue[1]  += 0.002
place_target_blue[0] += 0.002

place_target_yellow = list(position_yellow_peg)
place_target_yellow[2] += 0.08 
place_target_yellow[1] += 0.03 
place_target_yellow[0] += 0.03

# Inizializzazione tempi e variabili dinamiche
state_start_time = panda.simulationTime()
phase_start_pos = list(start_pos)  # Memorizza il punto di partenza della fase corrente
current_position = list(start_pos)
counter = 0 

# ==========================================
# DIZIONARIO TEMPI STANDARD (in secondi)
# Modifica questi valori per velocizzare o rallentare le fasi
# ==========================================
phase_durations = {
    "Approach": 7.0,
    "Pick": 3.0,
    "Move": 5.0,
    "Return": 4.0
}

while (t := panda.simulationTime()) < 60:
    stato = fsm.current_state()
    
    # Calcolo standardizzato di mov_alpha basato sullo stato corrente
    if stato in phase_durations:
        mov_alpha = min(1.0, (t - state_start_time) / phase_durations[stato])
    else:
        mov_alpha = 0.0
    
    # ==========================================
    # CICLO 0: ROSSO
    # ==========================================
    if counter == 0: 
        if stato == "Approach":
            sim.setJointTargetPosition(finger1, 0.04)
            sim.setJointTargetPosition(finger2, 0.04)
            
            current_position = lerp_3d(phase_start_pos, new_red, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)
            
            if mov_alpha >= 1.0:
                fsm.on_event("reached")
                state_start_time = t 
                phase_start_pos = list(current_position) 
                
        elif stato == "Pick":
            current_position = lerp_3d(phase_start_pos, pick_target_red, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0:
                sim.setObjectParent(red_torus, hand, 0)
                sim.setObjectParent(red_ring, red_torus, 0)
                sim.setJointTargetPosition(finger1, 0.00)
                sim.setJointTargetPosition(finger2, 0.02)
                
                fsm.on_event("picked")
                state_start_time = t
                phase_start_pos = list(current_position)
        
        elif stato == "Move":
            current_position = lerp_3d(phase_start_pos, place_target_red, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0:
                sim.setJointTargetPosition(finger1, 0.04)
                sim.setJointTargetPosition(finger2, 0.04)
                sim.setObjectParent(red_torus, red_cilinder, 0)
                
                final = lerp_3d(current_position, [position_red_peg[0], position_red_peg[1], position_red_peg[2]-0.15], mov_alpha)
                sim.setObjectPosition(red_torus, final, -1)
                
                fsm.on_event("arrived")
                fsm.on_event("placed")
                state_start_time = t
                phase_start_pos = list(current_position)

        elif stato == "Return":
            current_position = lerp_3d(phase_start_pos, base_pos, mov_alpha)
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0: 
                fsm.on_event("returned")
                counter += 1 
                state_start_time = t 
    
    # ==========================================
    # CICLO 1: BLU
    # ==========================================
    elif counter == 1:
        if stato == "Idle": 
            fsm.on_event("approach")
            state_start_time = t
            phase_start_pos = list(current_position)
    
        elif stato == "Approach": 
            sim.setJointTargetPosition(finger1, 0.04)
            sim.setJointTargetPosition(finger2, 0.04)
            
            current_position = lerp_3d(phase_start_pos, new_blue, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)
            
            if mov_alpha >= 1.0:
                fsm.on_event("reached")
                state_start_time = t 
                phase_start_pos = list(current_position)
        
        elif stato == "Pick":
            current_position = lerp_3d(phase_start_pos, pick_target_blue, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0:         
                sim.setObjectParent(blue_torus, hand, 0)
                sim.setObjectParent(blue_ring, blue_torus, 0)
                sim.setJointTargetPosition(finger1, 0.00)
                sim.setJointTargetPosition(finger2, 0.02)
                
                fsm.on_event("picked")
                state_start_time = t
                phase_start_pos = list(current_position)
        
        elif stato == "Move":
            current_position = lerp_3d(phase_start_pos, place_target_blue, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0:
                sim.setJointTargetPosition(finger1, 0.04)
                sim.setJointTargetPosition(finger2, 0.04)
                sim.setObjectParent(blue_torus, blue_cilinder, 0)
                
                final = lerp_3d(current_position, [position_blue_peg[0], position_blue_peg[1], position_blue_peg[2]-0.15], mov_alpha)
                sim.setObjectPosition(blue_torus, final, -1)
                sim.setObjectParent(blue_ring, blue_peg, 0)
                
                fsm.on_event("arrived")
                fsm.on_event("placed")
                state_start_time = t
                phase_start_pos = list(current_position)

        elif stato == "Return":
            current_position = lerp_3d(phase_start_pos, base_pos, mov_alpha)
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0: 
                fsm.on_event("returned")
                counter += 1 
                print("Counter: ", counter)
                state_start_time = t 

    # ==========================================
    # CICLO 2: GIALLO
    # ==========================================
    elif counter == 2:
        if stato == "Idle": 
            fsm.on_event("approach")
            state_start_time = t
            phase_start_pos = list(current_position)
    
        elif stato == "Approach": 
            sim.setJointTargetPosition(finger1, 0.04)
            sim.setJointTargetPosition(finger2, 0.04)
            
            current_position = lerp_3d(phase_start_pos, new_yellow, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)
            
            if mov_alpha >= 1.0:
                fsm.on_event("reached")
                state_start_time = t 
                phase_start_pos = list(current_position)
        
        elif stato == "Pick":
            current_position = lerp_3d(phase_start_pos, pick_target_yellow, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0:     
                sim.setObjectParent(yellow_torus, hand, 0)
                sim.setObjectParent(yellow_ring, yellow_torus, 0)
                sim.setJointTargetPosition(finger1, 0.00)
                sim.setJointTargetPosition(finger2, 0.02)
                
                fsm.on_event("picked")
                state_start_time = t
                phase_start_pos = list(current_position)
        
        elif stato == "Move":
            current_position = lerp_3d(phase_start_pos, place_target_yellow, mov_alpha)
            
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0:
                sim.setJointTargetPosition(finger1, 0.04)
                sim.setJointTargetPosition(finger2, 0.04)
                sim.setObjectParent(yellow_torus, yellow_cilinder, 0)
                
                final = lerp_3d(current_position, [position_yellow_peg[0], position_yellow_peg[1], position_yellow_peg[2]-0.15], mov_alpha)
                sim.setObjectPosition(yellow_torus, final, -1)
                sim.setObjectParent(yellow_ring, yellow_peg, 0)
                
                fsm.on_event("arrived")
                fsm.on_event("placed")
                state_start_time = t
                phase_start_pos = list(current_position)

        elif stato == "Return":
            current_position = lerp_3d(phase_start_pos, base_pos, mov_alpha)
            current_pose = current_position + start_orient
            sim.setObjectPose(target, current_pose, sim.handle_world)

            if mov_alpha >= 1.0: 
                fsm.on_event("returned")
                counter += 1 

    panda.stepSimulation()

panda.stopSimulation()