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



def lerp_3d(start: tuple[float, float, float], end: tuple[float, float, float], alpha: float) -> list[float]:
    """
    Esegue un'interpolazione lineare tra due punti 3D.
    """
    alpha = max(0.0, min(1.0, alpha))  # Clamp alpha to [0, 1]
    return [
        start[0] + alpha * (end[0] - start[0]),
        start[1] + alpha * (end[1] - start[1]),
        start[2] + alpha * (end[2] - start[2])
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
duration = 30.0
end_time = start_time + duration


fsm = PickAndPlaceFSM()
panda.startSimulation()
alpha = 1 
fsm.on_event("approach")  # Start the FSM

# Crea una copia indipendente delle coordinate!
new_red = list(position_red_ring)
new_red[2] += 0.2  # Altezza di approccio (20cm sopra l'anello, più sicuro per l'IK)
new_red[1] += 0.05

start_pose = sim.getObjectPose(target, sim.handle_world)
start_pos = start_pose[0:3]       # Estraiamo solo X, Y, Z
start_orient = start_pose[3:7]

# --- AGGIUNGI QUESTA RIGA ---
print(f"\nCoordinate target anello rosso: X={new_red[0]:.3f}, Y={new_red[1]:.3f}, Z={new_red[2]:.3f}")
# ---------------------------

fsm = PickAndPlaceFSM()
fsm.on_event("approach")  # Start the FSM

# Avvia la simulazione una sola volta
panda.startSimulation()

while (t := panda.simulationTime()) < 30:
    stato = fsm.current_state()
    alpha = (t - start_time) / duration
    print(f"Current FSM State: {stato} at time {t:.2f}s")
    
    if stato == "Approach":
        # 1. Apri il gripper
        sim.setJointTargetPosition(finger1, 0.04)
        sim.setJointTargetPosition(finger2, 0.04)
        
        # 2. Usa l'interpolazione lineare (lerp) per muovere il target FLUIDAMENTE
        mov_alpha = min(1.0, (t - start_time) / 3.0) 
        current_position = lerp_3d(start_pos, new_red, mov_alpha)
        current_pose = current_position + start_orient
        sim.setObjectPose(target, current_pose, sim.handle_world)
        
        # Se siamo arrivati a destinazione, cambia stato
        if mov_alpha >= 1.0:
            fsm.on_event("reached")
            
    elif stato == "Pick":
        # Qui potrai implementare la discesa verso l'anello
        mov_alpha = min(1.0, (t - start_time) / 5.0) 
        current_position = lerp_3d(current_position, [position_red_ring[0], position_red_ring[1],position_red_ring[2] + 0.005], mov_alpha)
        current_pose = current_position + start_orient
        sim.setObjectPose(target, current_pose, sim.handle_world)

        sim.setJointTargetPosition(finger1, 0.0)
        sim.setJointTargetPosition(finger2, 0.0)

        if mov_alpha >= 1.0:
            fsm.on_event("picked")    


    # Chiama lo step UNA sola volta per ciclo
    panda.stepSimulation()

panda.stopSimulation()