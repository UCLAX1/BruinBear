import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

class DepthListenerNode(Node):
    def __init__(self):
        super().__init__('depth_listener_node')
        # Create a subscriber for the depth topic
        self.subscription = self.create_subscription(
            Image,               # Message type
            '/camera/camera/depth/image_rect_raw',  # Topic name (match this to your topic)
            self.depth_image_callback, # Callback function when a message is received
            10)                  # QoS settings (10 means at least 10 messages are buffered)
        self.subscription  # Prevent unused variable warning

        # Create a CvBridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()

    def depth_image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Display depth image
            self.display_depth_image(depth_image)

            
        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV: {e}")

    def display_depth_image(self, depth_image):
        # Normalize depth image to 8-bit for display purposes
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        # Convert to 8-bit image for display (optional)
        depth_image_normalized = np.uint8(depth_image_normalized)
        # Example: Display the normalized depth image
        cv2.imshow("Depth Image", depth_image_normalized)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DepthListenerNode()  # Create the node
    rclpy.spin(node)  # Keep the node running
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
