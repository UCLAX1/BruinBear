
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
global gait_origin, gait



class gaitPublisher(Node):
  def __init__(self):
    super().__init__('gait_pub')
    self.publisher_ = self.create_publisher(String, 'gait_control', 10)
    timer_period = 1
    self.timer = self.create_timer(timer_period, self.pub_gait)
  
  def pub_gait(self, gait):
    msg = String()
    msg.data = gait
    self.publisher_.publish(msg)
    self.get_logger().info(f'Published gait: {msg.data}')



def main(args=None):
    rclpy.init()
    global gaitNode
    gaitNode = gaitPublisher()
    gait_origin = ""
    gait = "s"
    startTime = time.time()
    gaitNode.pub_gait(gait)
    time.sleep(3)
    gaitNode.pub_gait(gait)
    time.sleep(3)
    while True:
        curTime = time.time() - startTime
    
        if (curTime % 12 < 6):
            gait = "rnr"
        else:
            gait = "f"

        if (gait_origin != gait):
            gaitNode.pub_gait(gait)
            gait_origin = gait
            #time.sleep(3)
            #gaitNode.pub_gait(gait)
  

# Run the main function
if __name__ == "__main__":
    main()
    