import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import time
from hip import Hip
from knees import Knee

class JointPosSub(Node):
    """
    ROS 2 Subscriber Node to receive joint positions and update PID control
    """
    def __init__(self, leg_controller):
        super().__init__('joint_pos_sub')
        
        # Subscribe to the 'joint_positions' topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_positions',
            self.listener_callback,
            10  # Queue size
        )
        
        # Reference to LegController for updating targets
        self.leg_controller = leg_controller

    def listener_callback(self, msg):
        """
        Callback function to update LegController with new target positions
        """
        joint_positions = msg.data  # Extract joint position array

        # Ensure we received the correct number of positions
        if len(joint_positions) == 8:
            hip_targets = joint_positions[:4]  # First 4 values for hips
            knee_targets = joint_positions[4:] # Last 4 values for knees
            
            # Update the target positions for PID control
            self.leg_controller.update_target_positions(hip_targets, knee_targets)
            self.get_logger().info(f"Updated targets: Hips={hip_targets}, Knees={knee_targets}")
        else:
            self.get_logger().warn("Received invalid joint position data")


class LegController:
    """
    Manages hip and knee PID controllers and integrates with ROS 2
    """
    def __init__(self, dt=0.01):
        # Create 4 Hip joints
        self.hips = [
            Hip(kp=1.0, ki=1.0, kd=1.0, leg_id="front-left", dt=dt),
            Hip(kp=1.0, ki=1.0, kd=1.0, leg_id="front-right", dt=dt),
            Hip(kp=1.0, ki=1.0, kd=1.0, leg_id="back-left", dt=dt),
            Hip(kp=1.0, ki=1.0, kd=1.0, leg_id="back-right", dt=dt),
        ]
        
        # Create 4 Knee joints
        self.knees = [
            Knee(kp=1.0, ki=1.0, kd=1.0, leg_id="front-left", dt=dt),
            Knee(kp=1.0, ki=1.0, kd=1.0, leg_id="front-right", dt=dt),
            Knee(kp=1.0, ki=1.0, kd=1.0, leg_id="back-left", dt=dt),
            Knee(kp=1.0, ki=1.0, kd=1.0, leg_id="back-right", dt=dt),
        ]
        self.dt = dt
    
    def update_target_positions(self, hip_targets, knee_targets):
        """
        Update target positions for all hips and knees
        """
        for hip, target in zip(self.hips, hip_targets):
            hip.set_target_position(target)
        for knee, target in zip(self.knees, knee_targets):
            knee.set_target_position(target)
    
    def run(self):
        """
        Continuously run the PID controllers for all joints
        """
        while rclpy.ok():
            # Update motor power for all joints
            for hip in self.hips:
                hip.update_motor_power()
            
            for knee in self.knees:
                knee.update_motor_power()
            
            # Debug print
            print("\nJoint Status:")
            for hip in self.hips:
                print(hip)
            for knee in self.knees:
                print(knee)
            
            time.sleep(self.dt)


def main(args=None):
    """
    Entry point to initialize ROS 2 and run the leg controller
    """
    rclpy.init()
    
    # Initialize LegController
    leg_controller = LegController(dt=0.01)
    
    # Start ROS 2 Subscriber Node to receive joint positions
    joint_pos_sub = JointPosSub(leg_controller)
    
    # Create a MultiThreadedExecutor to run both the subscriber and leg controller
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(joint_pos_sub)
    
    # Run the PID loop in a separate thread
    from threading import Thread
    pid_thread = Thread(target=leg_controller.run, daemon=True)
    pid_thread.start()
    
    # Keep the subscriber running
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        joint_pos_sub.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
