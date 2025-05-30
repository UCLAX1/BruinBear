
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from collections import deque
global gait_origin, gait
global celldata
celldata = []
import controls.fsms as fsms

class gaitPublisher(Node):
  def __init__(self):
    super().__init__('gait_pub')
    self.publisher_ = self.create_publisher(String, 'gait_control', 10)
    self.subscription = self.create_subscription(
         Float32MultiArray,
         'imu_data', 
         self.listener_callback, 
         10
         )
    self.subscription = self.create_subscription(
       Float32MultiArray,
         'rangefinder_data', 
         self.listener_callback_obstacle, 
         10
         )

    self.subscription = self.create_subscription(
       Float32MultiArray,
       'postion_data',
       self.listener_callback_cell_data,
       10
       )
    
    # self.subscription
    #timer_period = 1
    #self.timer = self.create_timer(timer_period, self.pub_gait)
      
  def pub_gait(self, gait):
    msg = String()
    msg.data = gait
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
     global celldata
     celldata = msg.data
     

global average_position
average_position = []

def main():
    rclpy.init()
    global gaitNode
    gaitNode = gaitPublisher()
    
    
    global startTime
    startTime = time.time()
    global average_position
    global obstacle

    gait_origin = ""
    gait = "s"
    startTime = time.time()

    gaitNode.pub_gait(gait) #to wait for ros communication to get ready
    # time.sleep(3)

    state = "f"
    flag = True
    global cycle
    cycle = 0
    
    global celldata
    global fsm 
    # fsm = fsms.FSM()
    gaitNode.get_logger().info(str(len(celldata)))
    ping_pong = fsms.PingPong(logger=gaitNode.get_logger())
    # fsm.update()

    cycle_time = 200
    while True:
      # print(celldata)
      # print('loop happened')
      #curr_time = time.time()-startTime
      # print(case)
      rclpy.spin_once(gaitNode, timeout_sec=0)
      #update_average(imu_position)

      # print(len(celldata))
      #print(len(celldata[0]))
      
      # gait = fsm.update(curr_time, obstacle)
      if ping_pong.previousAction == 'f' or ping_pong.previousAction == 'b':
        cycle_time = 500
      elif ping_pong.previousAction == "lwr" or ping_pong.previousAction == "rwr":
        cycle_time = 2000
        
      if cycle % cycle_time == 0: 
        gait = ping_pong.update(celldata)

      # if (cycle % 10000 == 0 ):
      #     gaitNode.get_logger().info(f'Recieved obstacle: {obstacle}')
      #     gaitNode.get_logger().info(f"Recieved Cell Data:  {celldata}")
      #     gaitNode.get_logger().info(f'Publishing gait: {gait}')
      
      cycle += 1
      
      
      gaitNode.pub_gait(gait)
      time.sleep(0.001)

  

# Run the main function
if __name__ == "__main__":
  main()