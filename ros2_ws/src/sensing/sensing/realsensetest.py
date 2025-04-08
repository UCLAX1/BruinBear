import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from rclpy.executors import MultiThreadedExecutor

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
            self.get_center_point(depth_image)            
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

    def get_center_point(self, depth_image):
        grid_size = [16, 16]
        h, w = depth_image.shape
        cell_h, cell_w = h // grid_size[0], w // grid_size[1]
        # Iterate through grid cells
        depth_averages = np.zeros(grid_size)
        for i in range(grid_size[0]):
            for j in range(grid_size[1]):
                cell = depth_image[i * cell_h:(i + 1) * cell_h, j * cell_w:(j + 1) * cell_w]
                avg_depth = np.mean(cell)
                depth_averages[i, j] = avg_depth  # Store in the array

        center_point_depth = depth_image[depth_image.shape[0] // 2, depth_image.shape[1] // 2]
        print(center_point_depth)
        return center_point_depth

def main(args=None):
    rclpy.init(args=args)
    node = DepthListenerNode()  # Create the node
    # node.destroy_node()  # Ensure node is destroyed properly
    # rclpy.shutdown()  # Ensure proper shutdown of ROS2 context
    
    try:
        rclpy.spin(node)  # Spin the node to keep it alive
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown the node gracefully on exit
        rclpy.shutdown()
        pass


if __name__ == '__main__':
    main()
