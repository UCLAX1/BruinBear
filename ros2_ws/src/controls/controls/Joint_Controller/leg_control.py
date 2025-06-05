import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
try: 
    from hip import Hip
    from knee import Knee
    from roll import Roll
    from HardwareInterface import CanBus, Motor
except ImportError: 
    from controls.Joint_Controller.hip import Hip
    from controls.Joint_Controller.knee import Knee 
    from controls.Joint_Controller.roll import Roll
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
    
    # FLHip = Hip(1, bus, inverted=False)
    # FLKnee = Knee(2, bus)
    # FLRoll = Roll(3, bus)

    # FRHip = Hip(4, bus, inverted=True)
    # FRKnee = Knee(5, bus)
    # FRRoll = Roll(6, bus)

    # BRHip = Hip(7, bus, inverted=False)
    # BRKnee = Knee(8, bus)
    # BRRoll = Roll(9, bus)

    # BLHip = Hip(10, bus, inverted=True)
    # BLKnee = Knee(11, bus)
    # BLRoll = Roll(12, bus)

    joint_positions = [0] * 12
    
    cycle = 0
    last_time = time.time()
    
    while True:
        rclpy.spin_once(joint_pos_sub, timeout_sec=0)
        
        # if (cycle % 10000 == 0):
            # joint_pos_sub.get_logger().info(f"receiving positions: {hip_pos}, {knee_pos}, {roll_pos}")
            # joint_pos_sub.get_logger().info(f"setting ticks: {hip_ticks}, {knee_ticks}, {roll_ticks}")
            # joint_pos_sub.get_logger().info(f"setting power: {hip.motor_power}, {knee.motor_power}, {roll.motor_power}")
            # joint_pos_sub.get_logger().info(f"motors at pos: {hip.current_position}, {knee.current_position}, {roll.current_position}")
        # cycle+=1
        
        # FLHip.set_target_rad(joint_positions[0])
        # FLKnee.set_target_rad(joint_positions[4])
        # FLRoll.set_target_rad(joint_positions[8])

        # FRHip.set_target_rad(joint_positions[1])
        # FRKnee.set_target_rad(joint_positions[5])
        # FRRoll.set_target_rad(joint_positions[9])


        # BRHip.set_target_rad(joint_positions[2])
        # BRKnee.set_target_rad(joint_positions[6])
        # BRRoll.set_target_rad(joint_positions[10])

        # BLHip.set_target_rad(joint_positions[3])
        # BLKnee.set_target_rad(joint_positions[7])
        # BLRoll.set_target_rad(joint_positions[11])

        
        # FLKnee.update_motor_power()
        # FLHip.update_motor_power()
        # FLRoll.update_motor_power()

        # FRKnee.update_motor_power()
        # FRHip.update_motor_power()
        # FRRoll.update_motor_power()

        # BRKnee.update_motor_power()
        # BRHip.update_motor_power()
        # BRRoll.update_motor_power()

        # BLKnee.update_motor_power()
        # BLHip.update_motor_power()
        # BLRoll.update_motor_power()



        # Log cycle time every 1000 iterations
        cycle += 1
        if cycle % 1000 == 0:
            current_time = time.time()
            cycle_time = (current_time - last_time) / 1000  # Average time per cycle
            joint_pos_sub.get_logger().info(f"Average cycle time: {cycle_time:.6f} seconds")
            last_time = current_time

        
        
    
    


if __name__ == "__main__":
    main()
