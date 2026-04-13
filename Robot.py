import numpy as np
from numpy.linalg import matrix_rank
from numpy import array
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class Robot(object):
    def __init__(self, client, robot_name):
        # If there is an existing connection. Then open a connection
        self.client_ = client
        self.sim_ = self.client_.require('sim')

        # Robot frame
        self.frame = self.getHandler(robot_name)
        
        # https://manual.coppeliarobotics.com/en/simulation.htm#stepped
        self.sim_.setStepping(True)

    def getHandler(self, name):
        handler = self.sim_.getObject('/'+name)
        return handler

    def getHandlers(self, names):
        handlers = []
        for name in names:
            handler = self.getHandler(name)
            handlers.append(handler)

        return handlers

    def startSimulation(self):
        self.sim_.startSimulation()

    def stopSimulation(self):
        self.sim_.stopSimulation()

    def stepSimulation(self):
        self.sim_.step()

    def getSimulationTimeStep(self):
        return self.sim_.getSimulationTimeStep()

    def simulationTime(self):
        return self.sim_.getSimulationTime()

    def getMass(self):
        return self.sim_.getShapeMass(self.frame)

    def setJoints(self, joint_names):
        self.joints_ = self.getHandlers(joint_names)

    def getJointPosition(self):
        position = []
        for i in range(len(self.joints_)):
            position.append(self.sim_.getJointPosition(self.joints_[i]))
        return position

    def sendJointVelocities(self, vels):
        """
        Send motor velocities are specially useful to control joints such as wheels.
        :param vels:
        """
        for motor, vel in zip(self.joints_, vels):
            err_code = self.sim_.setJointTargetVelocity(motor, vel)

    def setPosition(self, position, relative_object=-1):
        if relative_object != -1:
            relative_object = self.get_handler(relative_object)
        self.sim_.setObjectPosition(self.frame, relative_object, position)

    def getPosition(self, relative_object=-1):
        """
        Get position relative to an object, -1 for global frame
        :param relative_object: id of the relative object
        :return: [x,y,z]
        """
        if relative_object != -1:
            relative_object = self.get_handler(relative_object)
        position = self.sim_.getObjectPosition(self.frame, relative_object)
        return np.array(position)

    def getOrientation(self, relative_object=-1):
        """
        Get orientation relative to an object, -1 for global frame
        eulerAngles: Euler angle [alpha beta gamma]
        :param relative_object: id of the relative object
        :return: [alpha beta gamma]
        """
        if relative_object != -1:
            relative_object = self.get_handler(relative_object)
        orientation = self.sim_.getObjectOrientation(self.frame, relative_object)
        return np.array(orientation)        

    def getVelocity(self, relative_object=-1):
        """
        Get velocity relative to an object, -1 for global frame
        :param relative_object:
        :return: [x,y,z], [roll, pitch, yaw]
        """
        if relative_object != -1:
            relative_object = self.get_handler(relative_object)
        velocity, omega = self.sim_.getObjectVelocity(self.frame)
        return array(velocity), array(omega)

    def getDistances(self):
        distances = []
        for sensor in self.sensors_:
            res, inrange, point, obj, surf_vector = self.sim_.readProximitySensor(sensor)
            distances.append((inrange, np.linalg.norm(point)))
        return distances

    def poseToMatrix(self,pose_list):
        position = np.array(pose_list[:3])
        quaternion = np.array(pose_list[3:])
        qw, qx, qy, qz = quaternion
        
        # Construct rotation matrix from quaternion
        rotation_matrix = np.array([
            [1 - 2*qy**2 - 2*qz**2, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw, 1 - 2*qx**2 - 2*qz**2, 2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1 - 2*qx**2 - 2*qy**2]
        ])
        
        # Combine rotation matrix and position into a transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = position
        
        return transformation_matrix

    def matrixToPose(self,matrix):
        # Extract rotation matrix and position from the transformation matrix
        rotation_matrix = matrix[:3, :3]
        # print(rotation_matrix)
        position = matrix[:3, 3]

        # Extract quaternion from rotation matrix
        # Calculate the sum of squares of the diagonal elements
        sum_of_squares = (1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2])

        # qw = np.sqrt(1 + rotation_matrix[0, 0] + rotation_matrix[1, 1] + rotation_matrix[2, 2]) / 2
        if sum_of_squares >= 0:
            qw = np.sqrt(sum_of_squares) / 2
        else:
            # Handle negative case (optional)
            print("Warning: Negative sum of squares encountered.")
            qw = 1
        print(qw)
        qx = (rotation_matrix[2, 1] - rotation_matrix[1, 2]) / (4*qw)
        qy = (rotation_matrix[0, 2] - rotation_matrix[2, 0]) / (4*qw)
        qz = (rotation_matrix[1, 0] - rotation_matrix[0, 1]) / (4*qw)

        return [position[0], position[1], position[2], qx, qy, qz, qw]
