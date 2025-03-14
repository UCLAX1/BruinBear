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
from roboticstoolbox import mstraj
import math
from controls.inverse_kinematics import solveIK
import controls.gaits as gaits
import controls.gait_publisher as publisher


startPos = [0] * 12
# startfront = solveIK([0,-0.3,0])
# startback = solveIK([0,-0.3,0], True)
# startPos = [startfront[0], startfront[0],
#             startfront[1], startfront[1],
#             startback[0], startback[0],
#             startback[1], startback[1],
#             0, 0, 0, 0]


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
      timer_period = 1
      # self.timer = self.create_timer(timer_period, self.pub_actuator_pos)
      #self.gait = ""
  
   def pub_actuator_pos(self, joint_positions):
      msg = Float32MultiArray()
      msg.data = joint_positions
      self.publisher_.publish(msg)
      #self.get_logger().info(f'received gait: {msg.data}')
   
   def listener_callback(self, msg):
      #self.gait
      #self.gait
      global gait
      gait = msg.data
      self.get_logger().info(f'received gait: {msg.data}')




# class jointPosSub(Node):

#     def __init__(self):
#         super().__init__('gait_sub')
#         self.subscription = self.create_subscription(String,'gait', self.listener_callback, 10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         # self.get_logger().info('recieved positions')
#         global gait
#         gait = msg.data


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
         case "leftInPlaceWithRoll"|"lwr":
            gaitTraj = gaits.TurnInPlaceWithRoll(turnRight=False, logger=trajNode.get_logger())
         case "rightInPlaceWithRoll"|"rwr":
            gaitTraj = gaits.TurnInPlaceWithRoll(turnRight=True, logger=trajNode.get_logger())

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
   #rclpy.spin(trajNode)
   #gait_publisher = publisher()
   #trajNode.listener_callback()
   #gait = trajNode.gait
   global gait
   gait = "f"
   #print(gait)


   walking = True

   #  gait = jointPosSub()
   #  global joints

   time.sleep(2)
   #wait for the model to drop in the sim

   while walking:
      rclpy.spin_once(trajNode, timeout_sec=0)
      trajNode.pub_actuator_pos(GeneratePosition())
   #rclpy.shutdown()

if __name__ == '__main__':
    main()