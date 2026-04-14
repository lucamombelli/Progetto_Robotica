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

#Ring handles
red_ring = sim.getObject(':/Red_ring')
yellow_ring = sim.getObject(':/Yellow_ring')
blue_ring = sim.getObject(':/Blue_ring')

# Peg Handle
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
duration = 50.0
end_time = start_time + duration


fsm = PickAndPlaceFSM()
fsm.on_event("approach")

panda.startSimulation()

# Salva le posizioni e gli orientamenti di partenza
start_pose = sim.getObjectPose(target, sim.handle_world)
start_pos = start_pose[0:3]       
start_orient = start_pose[3:7]

# Variabile per tracciare l'inizio di ogni singolo movimento
state_start_time = panda.simulationTime()

# Sta sopra di 20 cm dal'anello
new_red = list(position_red_ring)
new_red[2] += 0.2  

pick_target = list(position_red_ring)
# pick_target[2] += 0.003 # Scende fino al livello dell'anello
pick_target[1] += 0.05  # Si sposta sul raggio dell'anello

place_target = list(position_red_peg)
place_target[2] += 0.005 # Scende fino al livello dell'anello
place_target[1] += 0.1  # Si sposta sul raggio dell'anello

print(f"\nCoordinate target Approach: {new_red}")
print(f"Coordinate target Pick: {pick_target}")
print(f"Coordinate target Place: {place_target}")


# Variabile d'appoggio per salvare la posizione in aria prima di scendere
pos_prima_di_scendere = [0, 0, 0]

while (t := panda.simulationTime()) < 50:
    stato = fsm.current_state()
    
    if stato == "Approach":
        sim.setJointTargetPosition(finger1, 0.04)
        sim.setJointTargetPosition(finger2, 0.04)
        
        # Calcola il tempo passato SOLO in questo stato (es. 3 secondi totali)
        mov_alpha = min(1.0, (t - state_start_time) / 3.0) 
        current_position = lerp_3d(start_pos, new_red, mov_alpha)
        
        current_pose = current_position + start_orient
        sim.setObjectPose(target, current_pose, sim.handle_world)
        
        if mov_alpha >= 1.0:
            fsm.on_event("reached")
            # --- RESET DEI TEMPI PER LO STATO SUCCESSIVO ---
            state_start_time = t 
            # pos_prima_di_scendere = current_position # Salviamo il punto fisso da cui scendere
            
    elif stato == "Pick":
        # Discesa più rapida: 2 secondi
        mov_alpha = min(1.0, (t - state_start_time) / 2.0) 
        
        # Usiamo il punto fisso in aria come partenza, e il ring come arrivo
        current_position = lerp_3d(current_position, pick_target, mov_alpha)
        
        current_pose = current_position + start_orient
        sim.setObjectPose(target, current_pose, sim.handle_world)

        # CHIUDE LA PINZA SOLO QUANDO SEI ARRIVATO IN FONDO
        if mov_alpha >= 1.0:
            sim.setJointForce(finger1, 1000)
            sim.setJointForce(finger2, 1000)           

            sim.setJointTargetPosition(finger1, 0.0)
            sim.setJointTargetPosition(finger2, 0.0)
            
            fsm.on_event("picked")
            # Prepara il reset per lo stato "Move"
            state_start_time = t
    
    elif stato == "Move":
        mov_alpha = min(1.0, (t - state_start_time) / 10.0) 

        # Usiamo il punto fisso in aria come partenza, e il ring come arrivo
        current_position = lerp_3d(current_position, place_target, 0.009)
        
        current_pose = current_position + start_orient
        sim.setObjectPose(target, current_pose, sim.handle_world)

        if mov_alpha >= 1.0:
            sim.setJointTargetPosition(finger1, 0.04)
            sim.setJointTargetPosition(finger2, 0.04)
            
            fsm.on_event("arrived")
            # Prepara il reset per lo stato "Move"
            state_start_time = t

    panda.stepSimulation()

panda.stopSimulation()