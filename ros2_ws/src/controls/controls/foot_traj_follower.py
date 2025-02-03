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
from controls.inverse_kinematics import solveIK

r1 = 0.177
r2 = 0.177

gait = "r"

#initialize the trajectory and set the starting positions for the sim
#TRAJECTORY AND GAIT PARAMATERS
width = 0.14
width2 = 0.08
height = 0.03
dt = 0.01
totalTrajTime = 1
trotting = True
forwardStrokeTime = totalTrajTime/2 if trotting else totalTrajTime/4
backStrokeTime = totalTrajTime/2 if trotting else (3*totalTrajTime)/4
homeY = -0.3


viapoints = np.array([[0, homeY],
                    [-width/2, homeY],
                    [0, height+homeY],
                    [width/2, homeY],
                    [0, homeY]])

viapoints2 = np.array([[0, homeY],
                    [-width2/2, homeY],
                    [0, height+homeY],
                    [width2/2, homeY],
                    [0, homeY]])

viapoints_backward = np.array([[0, homeY],
                    [width/2, homeY],
                    [0, height+homeY],
                    [-width/2, homeY],
                    [0, homeY]])

time_segments = [backStrokeTime/2, forwardStrokeTime/2, forwardStrokeTime/2, backStrokeTime/2]


traj = mstraj(viapoints, dt, tacc = dt/4, tsegment = time_segments)
traj2 = mstraj(viapoints2, dt, tacc = dt/4, tsegment = time_segments)
traj_backward = mstraj(viapoints_backward, dt, tacc = dt/4, tsegment = time_segments)

positions = traj.q # number of positions is ~ totalTime/dt ish
positions_short = traj2.q
positions_backward = traj_backward.q


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

# class jointPosSub(Node):

#     def __init__(self):
#         super().__init__('gait_sub')
#         self.subscription = self.create_subscription(String,'gait', self.listener_callback, 10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         # self.get_logger().info('recieved positions')
#         global gait
#         gait = msg.data


def findTimeStep(t):
   t %= totalTrajTime
   return math.floor((t/totalTrajTime)*timeSteps)

def GeneratePosition(tFL, tFR, tBR, tBL):
   match gait:
      case "forward" | "f":
         posFL = solveIK(positions[findTimeStep(tFL)])
         posFR = solveIK(positions[findTimeStep(tFR)])
         posBR = solveIK(positions[findTimeStep(tBR)], True)
         posBL = solveIK(positions[findTimeStep(tBL)], True)
         
      case "right" | "r":
         posFL = solveIK(positions[findTimeStep(tFL)])
         posFR = solveIK(positions_short[findTimeStep(tFR)])
         posBR = solveIK(positions_short[findTimeStep(tBR)], True)
         posBL = solveIK(positions[findTimeStep(tBL)], True)
         

      case "left"|"l":
         posFL = solveIK(positions_short[findTimeStep(tFL)])
         posFR = solveIK(positions[findTimeStep(tFR)])
         posBR = solveIK(positions[findTimeStep(tBR)], True)
         posBL = solveIK(positions_short[findTimeStep(tBL)], True)
         

      case "backward"|"b":
         posFL = solveIK(positions_backward[findTimeStep(tFL)])
         posFR = solveIK(positions_backward[findTimeStep(tFR)])
         posBR = solveIK(positions_backward[findTimeStep(tBR)], True)
         posBL = solveIK(positions_backward[findTimeStep(tBL)], True)

   joint_positions = [posFL[0], posFR[0], posBR[0], posBL[0], 
                           posFL[1], posFR[1], posBR[1], posBL[1], 
                           0.0, 0.0, 0.0, 0.0]
   return joint_positions
   

def main(args=None):
    rclpy.init()
    trajNode = JointPosPublisher()
    walking = True

   #  gait = jointPosSub()
   #  global joints

    time.sleep(2)
   #wait for the model to drop in the sim

    startTime = time.time()
    while walking:
        curTime = time.time() - startTime
        tTraj = curTime % totalTrajTime

        #calculate the leg offsets for the gait
        tFL = (curTime)
        tBR = (curTime - totalTrajTime/4)
        #tFR = (curTime - totalTrajTime/2)
        tFR = (curTime- totalTrajTime/2)
        tBL = (curTime - 3*totalTrajTime/4)
        

  
        print('time', tBL)
        print('findTimeStep', findTimeStep(tBL))
        print('total time steps', timeSteps)

        # trajNode.get_logger().info("time step " + str(timeSteps))
        # trajNode.get_logger().info("pos at FL timestep " + str(positions[findTimeStep(tFL)]))
        trajNode.pub_actuator_pos(GeneratePosition(tFL, tFR, tBR, tBL))

if __name__ == '__main__':
    main()