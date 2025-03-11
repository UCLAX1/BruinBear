import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class HandRecogSub(Node):

    def __init__(self):
        super().__init__('hand_recog_sub')
        self.subscription = self.create_subscription(String,'recognition_result',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print("\033c") # disable if this causes problems, just clears the terminal
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    handRecogSub = HandRecogSub()

    rclpy.spin(handRecogSub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    handRecogSub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()