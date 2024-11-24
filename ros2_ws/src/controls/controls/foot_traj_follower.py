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

r1 = 0.177
r2 = 0.177

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
   return round((t/totalTrajTime)*timeSteps)

def solveIK(pos, backLeg=False):
    # The shoulder is at the origin and the target position is defined as being in the 3rd or 4th quadrants
    tx = pos[0]
    ty = pos[1]
    dT = np.sqrt(tx**2 + ty**2)
    th1 = np.arccos((r1**2 + r2**2 - dT**2)/(2 * r1 * r2))
    th2 = np.arccos((r1**2 + dT**2 - r2**2)/(2 * r1 * dT))
    if tx == 0:
        th3 = np.pi/2 + th2
    else:
        th3 = np.arctan(ty/tx) + th2
    # if abs(th3) < 0.0001:
    #     th3 = np.pi

    if tx < 0:
        th3 += np.deg2rad(90)
    else:
        th3 -= np.deg2rad(90)
    # if backLeg:
    #     th1 += np.deg2rad(202)
    #     # th1 += (180 - th3)
    #     # th3 *= 1.2
    # # th1 = np.pi - th1
    if th3 <= (np.pi/2):
        if th3 > 0:
            th3 *= -1
    if backLeg:
        th3 *= -1
        th1 -= np.pi
        # th1 *= 2
        # th1 -= (th3 - (np.pi/2))
        print(th3)
        print(th1)
    else:
        th1 = np.pi - th1
    return th1, th3

def main(args=None):
    rclpy.init()
    trajNode = JointPosPublisher()
    walking = True
    startTime = time.time()

    global timeSteps
    global dt
    global totalTrajTime
    
    width = 0.1
    height = 0.04
    dt = 0.01
    totalTrajTime = 10
    forwardStrokeTime = totalTrajTime/4
    backStrokeTime = (3*totalTrajTime)/4
    homeY = -0.12

    viapoints = np.array([[-width/2, homeY],
                          [0, height+homeY],
                          [width/2, homeY],
                          [0, homeY],
                          [-width/2, homeY]])

    time_segments = [forwardStrokeTime/2, forwardStrokeTime/2, backStrokeTime/2, backStrokeTime/2]
    traj = mstraj(viapoints, dt, tacc = dt/4, tsegment = time_segments)
    positions = traj.q # number of positions is ~ totalTime/dt ish
    timeSteps = len(positions)

    #STEP 1: generate trajectories
    while walking:
        curTime = time.time() - startTime
        tTraj = curTime % totalTrajTime

        #calculate the leg offsets for the gait
        tFL = (curTime) % totalTrajTime
        tBR = (curTime + totalTrajTime/4) % totalTrajTime
        tFR = (curTime + totalTrajTime/2) % totalTrajTime
        tBL = (curTime + 3*totalTrajTime/4) % totalTrajTime
        print('time', tBL)
        print('findTimeStep', findTimeStep(tBL))
        print('total time steps', timeSteps)

        posFL = solveIK(positions[findTimeStep(tFL)])
        posFR = solveIK(positions[findTimeStep(tFR)])
        posBR = solveIK(positions[findTimeStep(tBR)], True)
        posBL = solveIK(positions[findTimeStep(tBL)], True)

        #set to a standing position for now
        joint_positions = [posFL[0], posFR[0], posBR[0], posBL[0], 
                           posFL[1], posFR[1], posBR[1], posBL[1], 
                           0.0, 0.0, 0.0, 0.0]
        trajNode.pub_actuator_pos(joint_positions)

if __name__ == '__main__':
    main()