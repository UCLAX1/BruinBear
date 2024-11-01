import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class HandPosSub(Node):

    def __init__(self):
        super().__init__('hand_position_sub')
        self.subscription = self.create_subscription(
            String,
            'joint_position',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    handPosSub = HandPosSub()

    rclpy.spin(handPosSub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    handPosSub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()