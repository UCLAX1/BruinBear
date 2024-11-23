import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray



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

def main(args=None):
    rclpy.init()
    trajNode = JointPosPublisher()
    walking = True
    ### TODO: trajectory generation and following

    #STEP 1: generate trajectories
    while walking:

        #STEP 2: follow trajectories based on time

        #STEP 3: use IK to convert foot coordinates to joint positions

        #set to a standing position for now
        joint_positions = [-1.0, -1.0, 1.0, 1.0, 1.5, 1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 0.0]
        trajNode.pub_actuator_pos(joint_positions)

