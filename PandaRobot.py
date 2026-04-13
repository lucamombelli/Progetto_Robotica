from Robot import Robot
import numpy as np

class PandaRobot(Robot):
    def __init__(self, client, robot_name):
        super().__init__(client, robot_name)
 
        self.panda_joints_ = []
        for i in range(7):
            self.panda_joints_.append('Franka_joint' + str(i+1))
        
        print("Joint handles:", self.panda_joints_)
        
        self.setJoints(self.panda_joints_)
        
    def getStatus(self):
        print(self.getJointPosition()) 


    def trapezoidal_trajectory(self, t, t_acc, t_flat, t_dec, v_max):
        """ Generates a trapezoidal velocity profile.
        
        Parameters:
        t (numpy array): Time vector
        t_acc (float): Acceleration phase duration
        t_flat (float): Constant velocity phase duration
        t_dec (float): Deceleration phase duration
        v_max (float): Maximum velocity
        
        Returns:
        numpy array: Velocity at each time step
        """
        velocity = np.zeros_like(t)

        for i, time in enumerate(t):
            if time < t_acc:
                # Acceleration phase
                velocity[i] = (v_max / t_acc) * time
            elif time < t_acc + t_flat:
                # Constant velocity phase
                velocity[i] = v_max
            elif time < t_acc + t_flat + t_dec:
                # Deceleration phase
                velocity[i] = v_max - (v_max / t_dec) * (time - t_acc - t_flat)
            else:
                velocity[i] = 0  # After trajectory ends

        return velocity


# Example usage:
# client = some_simulation_client_instance
# panda = PandaRobot(client)
