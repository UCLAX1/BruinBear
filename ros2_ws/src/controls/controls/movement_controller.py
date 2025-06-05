
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from collections import deque
global gait_origin
global gait 
gait = 'sit'
global celldata
global cycle_time
cycle_time = 10
global previous_action
celldata = []
import controls.fsms as fsms

class gaitPublisher(Node):
  def __init__(self):
    super().__init__('gait_pub')
    self.get_logger().info("Initializing gaitPublisher node")
    self.publisher_ = self.create_publisher(String, 'gait_control', 10)
    self.imu_sub = self.create_subscription(
         Float32MultiArray,
         'imu_data', 
         self.listener_callback, 
         10
         )
    self.range_sub = self.create_subscription(
       Float32MultiArray,
         'rangefinder_data', 
         self.listener_callback_obstacle, 
         10
         )

    self.cam_sub = self.create_subscription(
       Float32MultiArray,
       '/position_data',
       self.listener_callback_cell_data,
       10
       )
    self.cycle = 0
    self.counter = 0
    self.ping_pong = fsms.PingPong(logger=self.get_logger())
    
    # self.subscription
    #timer_period = 1
    #self.timer = self.create_timer(timer_period, self.pub_gait)
      
  def pub_gait(self, message):
    msg = String()
    msg.data = message
    if self.counter % 10 == 0:
      self.get_logger().info(f'Publishing gait: {message}')
    self.publisher_.publish(msg)

  def listener_callback(self, msg):
      global imu_position
      imu_position = msg.data
      #self.get_logger().info(f'position: {msg.data}')
  def listener_callback_obstacle(self,msg):
     global obstacle
     obstacle = msg.data
     #self.get_logger().info(f'obstacle: {msg.data}')
  def listener_callback_cell_data(self,msg):
    #  print(msg.data)
    global celldata, cycle_time, gait
    celldata = msg.data

    # self.get_logger().info(f'Cycle Numer: {self.cycle}')
    # self.get_logger().info(f'Received celldata: {celldata}')

    # ping_pong = fsms.PingPong(logger=self.get_logger())

    if self.ping_pong.previousAction == 'f':
      cycle_time = 10
    elif self.ping_pong.previousAction == 'b':
      cycle_time = 50
    elif self.ping_pong.previousAction == "lwr" or self.ping_pong.previousAction == "rwr":
      cycle_time = 20
      
    if self.cycle % cycle_time == 0: 
      # gaitNode.get_logger().info(f'Before ping_pong.update. Current celldata: {celldata}')
      print()
      self.get_logger().info(f'Update Cycle Happened: {self.cycle}')

      gait = self.ping_pong.update(celldata)
      # gaitNode.get_logger().info(f'After ping_pong.update. New gait: {gait}')
    
    self.cycle += 1
    # self.get_logger().info(f'Publishing gait: {gait}')
    self.pub_gait(gait)
     

global average_position
average_position = []

def main():
    rclpy.init()
    global gait
    gaitNode = gaitPublisher()
    
    start = time.time()
    
    while time.time() - start < 1:
      gait = 'sit'
      gaitNode.pub_gait(gait)

    try:
      rclpy.spin(gaitNode)  
      pass
    finally:
      rclpy.shutdown()
      pass
    
  
  

# Run the main function
if __name__ == "__main__":
  main()