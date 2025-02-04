import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from roboticstoolbox import mstraj
import math
from controls.inverse_kinematics import solveIK

r1 = 0.177
r2 = 0.177

#initialize the trajectory and set the starting positions for the sim
#TRAJECTORY AND GAIT PARAMATERS
width = 0.15
height = 0.03
dt = 0.01
totalTrajTime = 3
trotting = True
forwardStrokeTime = totalTrajTime/2 if trotting else totalTrajTime/4
backStrokeTime = totalTrajTime/2 if trotting else (3*totalTrajTime)/4
homeY = -0.3
zPos = 0.1



#  if(self.state == 'get neutral'):
#                 self.neutralPos() # positionTargetX = 0.02, positionTargetY = .34, positionTargetZ = 0.0
#             elif self.state == 'INIT': 
#                 self.liftDiagonals('left', 'extended') # 0, 0.335, 0.1. For FL and BR
#             elif self.state == 'left_diagonals_lifted':
#                 self.extendDiagonal('left',1) # 0, 0.35, 0.1. FL and BR
#             elif self.state == 'left_extended1':
#                 self.extendDiagonal('right',1)  # 0, 0.35, 0.1. FR and BL
#             elif self.state == 'right_extended1':
#                 self.retractDiag('left',1) # 0, 0.35, 0. FL and BR
#             elif self.state == 'left_retracted1':
#                 self.retractDiag('right',2) # 0, 0.35, 0. FR and BL
#             elif self.state == 'right_retracted2':
#                 self.state = "INIT"


xPos = 0

viapoints = np.array([[xPos, homeY, 0],
                    [xPos, homeY, -width/2],
                    [xPos, height+homeY, 0],
                    [xPos, homeY, width/2],
                    [xPos, homeY, 0]]) # FL movement

time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/2, backStrokeTime/2]
traj = mstraj(viapoints, dt, tacc = dt/4, tsegment = time_segments)
positions = traj.q # number of positions is ~ totalTime/dt ish
timeSteps = len(positions)
#set the sim's starting position as the initial traj pos
startPos = [solveIK(positions[0])[0], solveIK(positions[0])[0],
            solveIK(positions[0], True)[0], solveIK(positions[0], True)[0],
            solveIK(positions[0])[1], solveIK(positions[0])[1],
            solveIK(positions[0], True)[1], solveIK(positions[0], True)[1],
            0, 0, 0, 0]

class JointPosPublisher(Node):
  def __init__(self):
    super().__init__('joint_pos_pub')
    self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_positions', 10)
    timer_period = 1
    self.timer = self.create_timer(timer_period, self.pub_actuator_pos)
  
  def pub_actuator_pos(self, joint_positions):
    msg = Float32MultiArray()
    msg.data = joint_positions
    self.publisher_.publish(msg)

def findTimeStep(t):
   t %= totalTrajTime
   return math.floor((t/totalTrajTime)*timeSteps)
   

def main(args=None):
    rclpy.init()
    trajNode = JointPosPublisher()
    walking = True

    time.sleep(1) #wait for the model to drop in the sim

    startTime = time.time()

    global timeSteps
    global dt
    global totalTrajTime
    
    trajHeight = 0.02
    trajWidth = trajHeight/5
    dt = 0.001
    totalTrajTime = 10
    forwardStrokeTime = totalTrajTime/4
    backStrokeTime = (3*totalTrajTime)/4
    homeY = 0.28

    viapoints = np.array([[0, homeY], [trajWidth/2, homeY+trajHeight], [0, homeY+trajHeight], [-trajWidth/2, homeY+trajHeight], [0, homeY]])

    time_segments = [forwardStrokeTime/2, backStrokeTime/2,  backStrokeTime/2, forwardStrokeTime/2]

    traj = mstraj(viapoints, dt, tacc = dt/4, tsegment = time_segments)
    positions = traj.q # number of positions is ~ totalTime/dt ish
    timeSteps = len(positions) - 1

    #STEP 1: generate trajectories
    while walking:
        curTime = time.time() - startTime
        
        tTraj = curTime % totalTrajTime

        #calculate the leg offsets for the gait
        tFL = (curTime)
        tBR = (curTime - totalTrajTime/4)
        tFR = (curTime - totalTrajTime/2)
        tBL = (curTime - 3*totalTrajTime/4)
        print('time', tBL)
        print('findTimeStep', findTimeStep(tBL))
        print('total time steps', timeSteps)

        posFL = solveIK(positions[findTimeStep(tFL)])
        posFR = solveIK(positions[findTimeStep(tFR)])
        posBR = solveIK(positions[findTimeStep(tBR)], True)
        posBL = solveIK(positions[findTimeStep(tBL)], True)

        joint_positions = [posFL[0], posFR[0], posBR[0], posBL[0], 
                           posFL[1], posFR[1], posBR[1], posBL[1], 
                           posFL[2], posFR[2], posBR[2], posBL[2]]
        trajNode.pub_actuator_pos(joint_positions)

if __name__ == '__main__':
    main()