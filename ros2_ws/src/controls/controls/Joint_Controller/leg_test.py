import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
try: 
    from hip import Hip
    from knee import Knee
    from HardwareInterface import CanBus, Motor
except ImportError: 
    from controls.Joint_Controller.hip import Hip
    from controls.Joint_Controller.knee import Knee 
    from controls.Joint_Controller.HardwareInterface import CanBus, Motor

global joint_positions
joint_positions = [0] * 12

class jointPosSub(Node):
    def __init__(self):
        super().__init__('joint_pos_sub')
        
        # Subscribe to the 'joint_positions' topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_positions',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        global joint_positions
        joint_positions = msg.data


def main(args=None):
    global joint_positions
    rclpy.init()
        
    # Start ROS 2 Subscriber Node to receive joint positions
    joint_pos_sub = jointPosSub()
    cycle = 0
    
    bus = CanBus()
    bus.start()
    
    knee = Knee(1, bus)
    hip = Hip(2, bus)
    
    while True:
        rclpy.spin_once(joint_pos_sub, timeout_sec=0)
        hip_pos = joint_positions[0]
        knee_pos = joint_positions[4]
        roll_pos = joint_positions[8]
        
        if (cycle % 10000 == 0):
            joint_pos_sub.get_logger().info(f"receiving positions: {hip_pos}, {knee_pos}, {roll_pos}")
        cycle+=1
        
        knee.set_target_rad(knee_pos)
        hip.set_target_rad(hip_pos)
        
        knee.update_motor_power()
        hip.update_motor_power()
        
        # time.sleep(0.2)
        # hip.update_motor_power()
        
        
    
    


if __name__ == "__main__":
    main()
