
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
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

def(self, min_value_position, min_value_distance):
        twist_msg = Twist()
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init()
    global gaitNode
    gaitNode = gaitPublisher()
    gait_origin = ""
    gait = "rnr"
    while True:
        if (gait_origin != gait):
            gaitNode.pub_gait(gait)
            time.sleep(5)
            #gait_origin = gait
  

# Run the main function
if __name__ == "__main__":
    main()
    

'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GaitPublisher(Node):
    def __init__(self):
        super().__init__('gait_pub')

        # Publisher for gait control
        self.publisher_ = self.create_publisher(String, 'gait_control', 10)

        # Store gait state as an instance variable
        self.gait = "f"  # Initial gait
        self.prev_gait = ""  # Track previous gait to avoid redundant publishing

        # Timer to periodically publish gait changes
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.pub_gait)

    def pub_gait(self):
        """Publishes gait command if it has changed."""
        #if self.gait != self.prev_gait:
        msg = String()
        msg.data = self.gait
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published gait: {msg.data}')
        #self.prev_gait = self.gait  # Update previous gait

def main(args=None):
    rclpy.init(args=args)
    node = GaitPublisher()
    node.pub_gait()
    print("publishing")
    #rclpy.spin(node)  # Proper ROS spinning
    # node.destroy_node()
    # rclpy.shutdown()

if __name__ == "__main__":
    main()
    '''