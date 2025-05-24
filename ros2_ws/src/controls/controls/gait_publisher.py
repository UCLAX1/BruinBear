
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
from collections import deque
global gait_origin, gait


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
    self.subscription
    #timer_period = 1
    #self.timer = self.create_timer(timer_period, self.pub_gait)
      
  def pub_gait(self, gait):
    msg = String()
    msg.data = gait
    self.publisher_.publish(msg)
   #  self.get_logger().info(f'Published gait: {msg.data}')

  def listener_callback(self, msg):
      global imu_position
      imu_position = msg.data
      #self.get_logger().info(f'position: {msg.data}')
  def listener_callback_obstacle(self,msg):
     global obstacle
     obstacle = msg.data
     #self.get_logger().info(f'obstacle: {msg.data}')

global average_position
average_position = []


# Parameters
n = 5  # number of values to average
recent_values = deque(maxlen=n)

def update_average(new_value):
    global average_position
    global cycle
    recent_values.append(np.array(new_value))  # ensure array type
    average_position = list(np.mean(recent_values, axis=0))
   #  if cycle%5000==0:
   #    gaitNode.get_logger().info(f'compare: {average_position[3]}')
    cycle+=1


def walk_forward_n_sec(n):
   global startPos
   global average_position
   global startTime
   #dy = imu_position[0] - startPos[0]
   #dx = imu_position[1]- startPos[1]
   #if abs(abs(imu_position[1])-abs(startPos[1]))>=n:
   if time.time()-startTime>=n:
      #gaitNode.get_logger().info(f'compare: {start_orientation,imu_position[3]}')
      return True
   return False


def angle_difference(a, b):
    """Returns the signed smallest difference between two angles (a - b), normalized to [-pi, pi]."""
    diff = a - b
    return (diff + math.pi) % (2 * math.pi) - math.pi

def turn_n_deg(n):
   global startPos
   global average_position
   n = (n-7) * (math.pi / 180)
   #if abs(abs(average_position[3])-abs(startPos[3]))>=n:
   #gaitNode.get_logger().info(f'compare: {imu_position[3],startPos[3]}')
   delta = angle_difference(startPos[3], imu_position[3])
   if abs(abs(delta))>=n:
      # gaitNode.get_logger().info(f'compare: {imu_position[3],startPos[3],delta}')
      return True
   return False

def turn_n_sec(n):
   global startTime
   global average_position
   #n = (n-10) * (math.pi / 180)
   #if abs(abs(average_position[3])-abs(startPos[3]))>=n:
   if time.time()-startTime>=n:
      #gaitNode.get_logger().info(f'compare: {abs(imu_position[3]),abs(startPos[3])}')
      return True
   return False

def main(args=None):
    rclpy.init()
    global gaitNode
    gaitNode = gaitPublisher()

    global imu_position
    imu_position = [0,0,0]
    global startPos
    startPos = [0,0,0]
    global startTime
    startTime = time.time()
    global average_position
    global obstacle
    #obstacle = [-1,1,-1]

    gait_origin = ""
    gait = "s"
    startTime = time.time()

    gaitNode.pub_gait(gait) #to wait for ros communication to get ready
    time.sleep(3)
    gaitNode.pub_gait(gait)
    time.sleep(3)

    state = "f"
    gait = "f"
    flag = True
    global cycle
    cycle = 0

    while True:
 
        rclpy.spin_once(gaitNode, timeout_sec=0)
        #update_average(imu_position)

      #   if (cycle % 1000 == 0 ):
      #       gaitNode.get_logger().info(f'Published obstacle: {obstacle}')
        cycle += 1
        

        match state:
         case "forward" | "f":
            gait = "f"
            #gaitNode.get_logger().info(f'inside forward: {"a"}')
            if obstacle[1]!=-1 and obstacle[1] <= 1:
               #gaitNode.get_logger().info(f'inside obstacle: {"a"}')
               state = "af"
               substate = "rnr"
               flag = True
               turningback = False
               isavoided = False
            # if flag:
            #    #startPos = average_position
            #    startPos = imu_position
            #    flag = False
            # gait = "f"
            # if walk_forward_n(3):
            #    state = "rnr"
            #    flag = True
         case "avoid_front_obstacle" | "af":
            match substate:
               case "rnr":
                  gait = "rnr"
                  if flag:
                     startPos = imu_position
                     startTime = time.time()
                     flag = False
                  if turn_n_sec(3):
                  #if turn_n_sec(3):
                     if isavoided:
                        state = "f"
                     
                     else:
                        substate = "f"
                        flag = True
                        turningback = True
                  
               case "f":
                  gait = "f"
                  if flag:
                     startPos = imu_position
                     startTime = time.time()
                     flag = False
                     gait = "f"
                  if walk_forward_n_sec(7):
                     if turningback:
                        substate = "lnr"
                     else:
                        substate = "rnr"
                        isavoided = True
                     flag = True

               case "lnr":
                  gait = "lnr"
                  if flag:
                     startPos = imu_position
                     startTime = time.time()
                     flag = False
                  if turn_n_sec(6):
                  #if turn_n_sec(6):
                     substate = "f"
                     flag = True
                     turningback = False

         case "rightWalk" | "rw":
            continue
         case "leftWalk"|"lw":
            continue
         case "backward"|"b":
            continue
         case "trot" | "t":
            continue
         case "rightInPlaceNoRoll" | "rnr":
            gait = "rnr"
            # if cycle%100==0:
            #    gaitNode.get_logger().info(f'compare: {imu_position[3]}')
            # cycle+=1
            
            # if flag:
            #    #startPos = average_position
            #    startPos = imu_position
            #    flag = False
            # gait = "rnr"
            # if turn_n_deg(180):
            #    state = "f"
            #    flag = True
         case "leftInPlaceNoRoll"|"lnr":
            continue
         case "standing" | "s":
            continue
         case "leftInPlaceWithRoll"|"lwr":
            continue
         case "rightInPlaceWithRoll"|"rwr":
            continue
           
        if (gait_origin != gait):
            gaitNode.pub_gait(gait)
            gait_origin = gait

  

# Run the main function
if __name__ == "__main__":
    main()
    