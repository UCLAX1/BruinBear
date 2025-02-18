import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import time
from roboticstoolbox import mstraj
import math

class Gait:
    def __init__(self, logger = None):
        self.logger = logger
        self.totalTrajTime = 0
        self.timeSteps = 0
    def getPos(self, time):
        raise NotImplementedError("This method should be overridden by subclasses")
    def findTimeStep(self, t):
        t %= self.totalTrajTime
        return math.floor((t/self.totalTrajTime)*self.timeSteps)
    def log(self, msg):
        if self.logger:
            self.logger.info(str(msg))

class Stand(Gait):
    def __init__(self, logger = None):
        super().__init__(logger)

    def getPos(self, time):
        return [0, -0.3, 0] * 4

class Walk(Gait):
    def __init__(self, logger = None):
        super().__init__(logger)
        width = 0.15
        height = 0.03
        dt = 0.01
        self.totalTrajTime = 1
        trotting = True
        forwardStrokeTime = self.totalTrajTime/4
        backStrokeTime = (3*self.totalTrajTime)/4
        homeY = -0.3
        viapoints = np.array([[0, homeY, 0],
                            [-width/2, homeY, 0],
                            [0, height+homeY, 0],
                            [3*width/4, height+homeY, 0],
                            [width/2, homeY, 0],
                            [0, homeY, 0]])

        # viapoints = np.array([[0, homeY, 0],
        #                     [-width/2, homeY, 0],
        #                     [0, height+homeY, 0],
        #                     [width/2, homeY, 0],
        #                     [0, homeY, 0]])
        
        time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/4, forwardStrokeTime/4, backStrokeTime/2]
        # time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/2, backStrokeTime/2]
        traj = mstraj(viapoints, dt, tacc = dt/4, tsegment = time_segments)
        self.positions = traj.q
        self.timeSteps = len(self.positions)

    def getPos(self, time):
        tFL = (time)
        tFR = (time - self.totalTrajTime/2)
        tBR = (time)
        tBL = (time - self.totalTrajTime/2)
    # def getPos(self, time):

    #     tFL = (time)
    #     tBR = (time - self.totalTrajTime/4)
    #     tFR = (time- self.totalTrajTime/2)  
    #     tBL = (time - 3*self.totalTrajTime/4)
    
        return [self.positions[self.findTimeStep(tFL)],
                self.positions[self.findTimeStep(tFR)],
                self.positions[self.findTimeStep(tBR)],
                self.positions[self.findTimeStep(tBL)]]


class WalkTurn(Gait):
    def __init__(self, turnRight, logger = None):
        super().__init__(logger)
        self.turnRight = turnRight
        width_long = 0.14
        width_short = 0.08
        height = 0.03
        dt = 0.01
        self.totalTrajTime = 1
        forwardStrokeTime = self.totalTrajTime/4
        backStrokeTime = (3*self.totalTrajTime)/4
        homeY = -0.3
        viapoints_long = np.array([[0, homeY, 0],
                            [-width_long/2, homeY, 0],
                            [0, height+homeY, 0],
                            [3*width_long/4, height+homeY, 0],
                            [width_long/2, homeY, 0],
                            [0, homeY, 0]])

        viapoints_short = np.array([[0, homeY, 0],
                            [-width_short/2, homeY, 0],
                            [0, height+homeY, 0],
                            [3*width_short/4, height+homeY, 0],
                            [width_short/2, homeY, 0],
                            [0, homeY, 0]])

        time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/4, forwardStrokeTime/4, backStrokeTime/2]

        traj_long = mstraj(viapoints_long, dt, tacc = dt/4, tsegment = time_segments)
        traj_short = mstraj(viapoints_short, dt, tacc = dt/4, tsegment = time_segments)
        self.positions_long = traj_long.q
        self.positions_short = traj_short.q
        self.timeSteps = len(self.positions_long)

    def getPos(self, time):
        tFL = (time)
        tFR = (time - self.totalTrajTime/2)
        tBR = (time)
        tBL = (time - self.totalTrajTime/2)
    
        if (self.turnRight):
            return [self.positions_long[self.findTimeStep(tFL)],
                    self.positions_short[self.findTimeStep(tFR)],
                    self.positions_short[self.findTimeStep(tBR)],
                    self.positions_long[self.findTimeStep(tBL)]]
        else:
            return [self.positions_short[self.findTimeStep(tFL)],
                    self.positions_long[self.findTimeStep(tFR)],
                    self.positions_long[self.findTimeStep(tBR)],
                    self.positions_short[self.findTimeStep(tBL)]]
        

class TurnInPlaceNoRoll(Gait):
    def __init__(self, turnRight, logger = None):
        super().__init__(logger)
        self.turnRight = turnRight
        width = 0.08
        height = 0.02
        dt = 0.01
        self.totalTrajTime = 1
        forwardStrokeTime = self.totalTrajTime/2
        backStrokeTime = self.totalTrajTime/2
        homeY = -0.3
        viapoints_forward = np.array([[0, homeY, 0],
                            [-width/2, homeY, 0],
                            [0, height+homeY, 0],
                            [3*width/4, height+homeY, 0],
                            [width/2, homeY, 0],
                            [0, homeY, 0]])
        
        viapoints_backward = np.array([[0, homeY, 0],
                            [width/2, homeY, 0],
                            [0, height+homeY, 0],
                            [-3*width/4, height+homeY, 0],
                            [-width/2, homeY, 0],
                            [0, homeY, 0]])


        time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/4, forwardStrokeTime/4, backStrokeTime/2]

        traj_forward = mstraj(viapoints_forward, dt, tacc = dt/4, tsegment = time_segments)
        traj_backward = mstraj(viapoints_backward, dt, tacc = dt/4, tsegment = time_segments)
        self.positions_forward = traj_forward.q
        self.positions_backward = traj_backward.q
        self.timeSteps = len(self.positions_backward)

    def getPos(self, time):
        tFL = (time)
        tFR = (time - self.totalTrajTime/2)
        tBR = (time)
        tBL = (time - self.totalTrajTime/2)
    
        if (self.turnRight):
            return [self.positions_forward[self.findTimeStep(tFL)],
                    self.positions_backward[self.findTimeStep(tFR)],
                    self.positions_backward[self.findTimeStep(tBR)],
                    self.positions_forward[self.findTimeStep(tBL)]]
        else:
            return [self.positions_backward[self.findTimeStep(tFL)],
                    self.positions_forward[self.findTimeStep(tFR)],
                    self.positions_forward[self.findTimeStep(tBR)],
                    self.positions_backward[self.findTimeStep(tBL)]]
        


class WalkBackward(Gait):
    def __init__(self, logger = None):
        super().__init__(logger)
        width = 0.15
        height = 0.03
        dt = 0.01
        self.totalTrajTime = 2
        trotting = True
        forwardStrokeTime = self.totalTrajTime/4
        backStrokeTime = (3*self.totalTrajTime)/4
        homeY = -0.3
        viapoints = np.array([[0, homeY, 0],
                            [width/2, homeY, 0],
                            [0, height+homeY, 0],
                            [-3*width/4, height+homeY, 0],
                            [-width/2, homeY, 0],
                            [0, homeY, 0]])

        # viapoints = np.array([[0, homeY, 0],
        #                     [-width/2, homeY, 0],
        #                     [0, height+homeY, 0],
        #                     [width/2, homeY, 0],
        #                     [0, homeY, 0]])
        
        time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/4, forwardStrokeTime/4, backStrokeTime/2]
        # time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/2, backStrokeTime/2]
        traj = mstraj(viapoints, dt, tacc = dt/4, tsegment = time_segments)
        self.positions = traj.q
        self.timeSteps = len(self.positions)

    def getPos(self, time):
        tFL = (time)
        tFR = (time - self.totalTrajTime/2)
        tBR = (time)
        tBL = (time - self.totalTrajTime/2)
    # def getPos(self, time):

    #     tFL = (time)
    #     tBR = (time - self.totalTrajTime/4)
    #     tFR = (time- self.totalTrajTime/2)  
    #     tBL = (time - 3*self.totalTrajTime/4)
    
        return [self.positions[self.findTimeStep(tFL)],
                self.positions[self.findTimeStep(tFR)],
                self.positions[self.findTimeStep(tBR)],
                self.positions[self.findTimeStep(tBL)]]

class Trot(Gait):
    def __init__(self, logger = None):
        super().__init__(logger)
        width = 0.13
        height = 0.03
        dt = 0.01
        self.totalTrajTime = 1
        trotting = True
        forwardStrokeTime = self.totalTrajTime/2
        backStrokeTime = self.totalTrajTime/2
        homeY = -0.3
        viapoints = np.array([[0, homeY, 0],
                            [-width/2, homeY, 0],
                            [0, height+homeY, 0],
                            [3*width/4, height+homeY, 0],
                            [width/2, homeY, 0],
                            [0, homeY, 0]])
        
        time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/4, forwardStrokeTime/4, backStrokeTime/2]
        traj = mstraj(viapoints, dt, tacc = dt/4, tsegment = time_segments)
        self.positions = traj.q
        self.timeSteps = len(self.positions)

    def getPos(self, time):
        tFL = (time)
        tFR = (time - self.totalTrajTime/2)
        tBR = (time)
        tBL = (time - self.totalTrajTime/2)
    
        return [self.positions[self.findTimeStep(tFL)],
                self.positions[self.findTimeStep(tFR)],
                self.positions[self.findTimeStep(tBR)],
                self.positions[self.findTimeStep(tBL)]]

class TurnInPlaceWithRoll(Gait):
    def __init__(self, turnRight, logger = None):
        super().__init__(logger)
        self.turnRight = turnRight
        width = 0.1
        height = 0.03
        dt = 0.01
        self.totalTrajTime = 1
        trotting = True
        forwardStrokeTime = self.totalTrajTime/2
        backStrokeTime = self.totalTrajTime/2
        homeY = -0.3
        zPos = 0.1


        xPos = 0
        viapoints1 = np.array([[xPos, homeY, 0],
                            [xPos, homeY,   width/2],
                            [xPos, height+homeY, 0],
                            [xPos, homeY, -width/2],
                            [xPos, homeY, 0]])
        viapoints2 = np.array([[xPos, homeY, 0],
                            [xPos, homeY,   -width/2],
                            [xPos, height+homeY, 0],
                            [xPos, homeY, width/2],
                            [xPos, homeY, 0]])
        
        time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/2, backStrokeTime/2]
        traj1 = mstraj(viapoints1, dt, tacc = dt/4, tsegment = time_segments)
        traj2 = mstraj(viapoints2, dt, tacc = dt/4, tsegment = time_segments)
        self.positions1 = traj1.q 
        self.positions2 = traj2.q 
        self.timeSteps = len(self.positions1)

    def getPos(self, time):
        tFL = (time)
        tFR = (time - self.totalTrajTime/2)
        tBR = (time)
        tBL = (time - self.totalTrajTime/2)
        
        if self.turnRight:
            return [self.positions1[self.findTimeStep(tFL)],
                self.positions2[self.findTimeStep(tFR)],
                self.positions1[self.findTimeStep(tBR)],
                self.positions2[self.findTimeStep(tBL)]]
        else:
            return [self.positions2[self.findTimeStep(tFL)],
                    self.positions1[self.findTimeStep(tFR)],
                    self.positions2[self.findTimeStep(tBR)],
                    self.positions1[self.findTimeStep(tBL)]]

