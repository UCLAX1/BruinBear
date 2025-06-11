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
import scipy

TARGET = None

# Patch scipy.randn if missing
if not hasattr(scipy, 'randn'):
    scipy.randn = np.random.randn

from roboticstoolbox import mstraj

class Gait:
    def __init__(self, logger = None):
        global TARGET
        self.logger = logger
        self.totalTrajTime = 0
        self.timeSteps = 0
        self.dt = 0.01
        
        # create a smooth path from the startsFrom position (last set target) to the start of the traj
        self.startsFrom = TARGET
        self.startUpTime = 0
        self.startTrajPos = [[] for _ in range(4)]
        self.init_start = False
        
    def init_start_traj(self):
        global TARGET
        self.init_start = True
        if self.startsFrom is not None:
            startsAt = self.update(0)
            for i in range(4):
                viapoints = np.array([self.startsFrom[i], startsAt[i]])
                speed = 0.1  # meters per second
                duration = self.distance(self.startsFrom[i], startsAt[i]) / speed
                # Make sure duration is at least one dt (so mstraj doesn’t fail)
                duration = max(duration, self.dt)
                
                
                traj = mstraj(viapoints, self.dt, tacc = self.dt/4, tsegment=[duration])
                self.startTrajPos[i] = traj.q
                self.startUpTime = max(self.startUpTime, duration)
        
    def getPos(self, time):
        global TARGET
        
        if not self.init_start:
            self.init_start_traj()
        time = abs(time)
        if time < self.startUpTime:
            for i in range(4):
                # interpolate between the start position and the target position
                if len(self.startTrajPos[i]) > 0:
                    step = int ((time/self.startUpTime) * len(self.startTrajPos[i]))
                    if step < len(self.startTrajPos[i]):
                        TARGET[i] = self.startTrajPos[i][step]
                    else:
                        TARGET[i] = self.startTrajPos[i][-1]
        else: #regular trajectory following
            TARGET = self.update(time - self.startUpTime)
        return TARGET
    
    def update(self, time):
        raise NotImplementedError("This method should be overridden by subclasses")
    
    def findTimeStep(self, t):
        t %= self.totalTrajTime
        return math.floor((t/self.totalTrajTime)*self.timeSteps)
    
    def distance(self, pos1, pos2):
        return np.linalg.norm(np.array(pos1) - np.array(pos2))
    
    def log(self, msg):
        if self.logger:
            self.logger.info(str(msg))

class Stand(Gait):
    def __init__(self, logger = None):
        super().__init__(logger)

    def update(self, time):
        return [[0, -0.3, 0]] * 4
    
class Sit(Gait):
    def __init__(self, logger = None):
        super().__init__(logger)

    def update(self, time):
        return [[0, -0.15, 0]] * 4


class Walk(Gait):
    def __init__(self, logger = None):
        super().__init__(logger)
        width = 0.15
        height = 0.03
        self.dt = 0.01
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
        
        time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/4, forwardStrokeTime/4, backStrokeTime/2]
        traj = mstraj(viapoints, self.dt, tacc = self.dt/4, tsegment = time_segments)
        self.positions = traj.q
        self.timeSteps = len(self.positions)

    def update(self, time):
        tFL = (time)
        tFR = (time - self.totalTrajTime/2)
        tBR = (time)
        tBL = (time - self.totalTrajTime/2)
    
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
        self.dt = 0.01
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

        traj_long = mstraj(viapoints_long, self.dt, tacc = self.dt/4, tsegment = time_segments)
        traj_short = mstraj(viapoints_short, self.dt, tacc = self.dt/4, tsegment = time_segments)
        self.positions_long = traj_long.q
        self.positions_short = traj_short.q
        self.timeSteps = len(self.positions_long)

    def update(self, time):
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
        self.dt = 0.01
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

        traj_forward = mstraj(viapoints_forward, self.dt, tacc = self.dt/4, tsegment = time_segments)
        traj_backward = mstraj(viapoints_backward, self.dt, tacc = self.dt/4, tsegment = time_segments)
        self.positions_forward = traj_forward.q
        self.positions_backward = traj_backward.q
        self.timeSteps = len(self.positions_backward)

    def update(self, time):
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
        self.dt = 0.01
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
        
        time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/4, forwardStrokeTime/4, backStrokeTime/2]
        traj = mstraj(viapoints, self.dt, tacc = self.dt/4, tsegment = time_segments)
        self.positions = traj.q
        self.timeSteps = len(self.positions)

    def update(self, time):
        tFL = (time)
        tFR = (time - self.totalTrajTime/2)
        tBR = (time)
        tBL = (time - self.totalTrajTime/2)
    
        return [self.positions[self.findTimeStep(tFL)],
                self.positions[self.findTimeStep(tFR)],
                self.positions[self.findTimeStep(tBR)],
                self.positions[self.findTimeStep(tBL)]]

class Trot(Gait):
    def __init__(self, logger = None):
        super().__init__(logger)
        width = 0.13
        height = 0.03
        self.dt = 0.01
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
        traj = mstraj(viapoints, self.dt, tacc = self.dt/4, tsegment = time_segments)
        self.positions = traj.q
        self.timeSteps = len(self.positions)

    def update(self, time):
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
        self.dt = 0.01
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
        traj1 = mstraj(viapoints1, self.dt, tacc = self.dt/4, tsegment = time_segments)
        traj2 = mstraj(viapoints2, self.dt, tacc = self.dt/4, tsegment = time_segments)
        self.positions1 = traj1.q 
        self.positions2 = traj2.q 
        self.timeSteps = len(self.positions1)

    def update(self, time):
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

