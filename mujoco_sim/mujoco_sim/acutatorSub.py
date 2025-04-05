import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('actuator_pos_subscriber')
        
        # Create a subscriber to listen to joint states
        self.subscription = self.create_subscription(
            JointState,
            'actuator_postions',  # The topic name for joint states
            self.listener_callback,
            10
        )
        
        self.get_logger().info("Actuator Position Subscriber has been started.")

    def listener_callback(self, msg):
        # This is the callback function that is called when a new message arrives
        self.get_logger().info(f"Received joint states: {msg.position}")
        
        # Optionally, print out more detailed joint state info:
        for joint_name, position in zip(msg.name, msg.position):
            self.get_logger().info(f"Actuator: {joint_name}, Positions: {position}, Velocity: {msg.velocity[msg.name.index(joint_name)]}")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()