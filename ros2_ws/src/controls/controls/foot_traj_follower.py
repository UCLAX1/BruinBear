import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from std_msgs.msg import Float32MultiArray, String
import time
# from roboticstoolbox import mstraj
import math
from controls.inverse_kinematics import solveIK
import controls.gaits as gaits
import controls.gait_publisher as publisher


#startPos = [0] * 12

pos_start = gaits.Sit().getPos(0)  # Get the initial position for standing

posFL_start = solveIK(pos_start[0])
posFR_start = solveIK(pos_start[1])
posBR_start = solveIK(pos_start[2], True)
posBL_start = solveIK(pos_start[3], True)
startPos = [posFL_start[0], posFR_start[0], posBR_start[0], posBL_start[0], 
                        posFL_start[1], posFR_start[1], posBR_start[1], posBL_start[1], 
                        posFL_start[2], posFR_start[2], posBR_start[2], posBL_start[2]]

class JointPosPublisher(Node):
   def __init__(self):
      super().__init__('joint_pos_pub')
      self.publisher_ = self.create_publisher(Float32MultiArray, 'joint_positions', 10)
      self.subscription = self.create_subscription(
         String,
         'gait_control', 
         self.listener_callback, 
         10
         )
      self.subscription
      #timer_period = 1
      # self.timer = self.create_timer(timer_period, self.pub_actuator_pos)
      #self.gait = ""
  
   def pub_actuator_pos(self, joint_positions):
      msg = Float32MultiArray()
      msg.data = joint_positions
      self.publisher_.publish(msg)
      #self.get_logger().info(f'published pos: {msg.data}')
   
   def listener_callback(self, msg):
      global gait
      gait = msg.data
      # self.get_logger().info(f'received gait: {msg.data}')



startTime = time.time()
#gaitTraj = gaits.Walk()
global prevGait
prevGait = ""
cycle = 0




def GeneratePosition():
   global gait, startTime, trajNode, gaitTraj, cycle, prevGait
   
   if prevGait != gait:
      prevGait = gait
      startTime = time.time()
      
      match gait:
         case "forward" | "f":
            gaitTraj = gaits.Walk(trajNode.get_logger())
         case "rightWalk" | "rw":
            gaitTraj = gaits.WalkTurn(turnRight=True, logger=trajNode.get_logger())
         case "leftWalk"|"lw":
            gaitTraj = gaits.WalkTurn(turnRight=False, logger=trajNode.get_logger())
         case "backward"|"b":
            gaitTraj = gaits.WalkBackward(trajNode.get_logger())
         case "trot" | "t":
            gaitTraj = gaits.Trot(trajNode.get_logger())
         case "rightInPlaceNoRoll" | "rnr":
            gaitTraj = gaits.TurnInPlaceNoRoll(turnRight=True, logger=trajNode.get_logger())
         case "leftInPlaceNoRoll"|"lnr":
            gaitTraj = gaits.TurnInPlaceNoRoll(turnRight=False, logger=trajNode.get_logger())
         case "standing" | "s":
            gaitTraj = gaits.Stand(trajNode.get_logger())
         case "leftInPlaceWithRoll"|"lwr":
            gaitTraj = gaits.TurnInPlaceWithRoll(turnRight=False, logger=trajNode.get_logger())
         case "rightInPlaceWithRoll"|"rwr":
            gaitTraj = gaits.TurnInPlaceWithRoll(turnRight=True, logger=trajNode.get_logger())
         case "sit" | "si":
            gaitTraj = gaits.Sit(trajNode.get_logger(), logger=trajNode.get_logger())

   curTime = time.time() - startTime
   pos = gaitTraj.getPos(curTime)
   
   posFL = solveIK(pos[0])
   posFR = solveIK(pos[1])
   posBR = solveIK(pos[2], True)
   posBL = solveIK(pos[3], True)
   joint_positions = [posFL[0], posFR[0], posBR[0], posBL[0], 
                           posFL[1], posFR[1], posBR[1], posBL[1], 
                           posFL[2], posFR[2], posBR[2], posBL[2]]
   
   # if (cycle % 100 == 0 ):
   #    # trajNode.get_logger().info(str(gaitTraj))
   #    #trajNode.get_logger().info(str(posFL))
   # cycle += 1
   
   return joint_positions
   

def main(args=None):
   rclpy.init()
   global trajNode 
   trajNode = JointPosPublisher()
   global gait
   gait = "f"

   walking = True

   time.sleep(2)
   #wait for the model to drop in the sim

   while walking:
      rclpy.spin_once(trajNode, timeout_sec=0)
      trajNode.pub_actuator_pos(GeneratePosition())
   #rclpy.shutdown()

if __name__ == '__main__':
    main()