import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from matplotlib import pyplot as plt

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'depth_image', 10)

        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.align_to = rs.stream.color
        self.align = rs.align(self.align_to)

        # Start camera stream
        try:
            self.pipeline.start(self.config)
            self.get_logger().info("RealSense camera started successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to start RealSense camera: {e}")
            rclpy.shutdown()

        # Timer to pull frames and publish data
        self.timer = self.create_timer(0.1, self.publish_depth_image)

    def get_depth_image_array(self):

        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
            
        if not depth_frame:
            self.get_logger().warn("No depth frame available.")

        # Convert depth frame to a numpy array
        depth_image = np.asanyarray(depth_frame.get_data())

        return depth_image

    #get stream of images and continously publish
    def get_flow_of_images(self):
        pass

    def get_center_point(self, depth_image):
        # Define grid size
        grid_size = [4, 4]  # Number of rows and columns
        h, w = depth_image.shape
        cell_h, cell_w = h // grid_size[0], w // grid_size[1]
        # Iterate through grid cells

        depth_averages = np.zeros(grid_size)
        for i in range(grid_size[0]):
            for j in range(grid_size[1]):
                cell = depth_image[i * cell_h:(i + 1) * cell_h, j * cell_w:(j + 1) * cell_w]
                avg_depth = np.mean(cell)
                depth_averages[i, j] = avg_depth  # Store in the array


        center_point_depth = depth_image[depth_image.shape[0]//2, depth_image.shape[1]//2]

        return center_point_depth
    


    def blur_depth_image(self, depth_image, h, w):
        #kernel = np.ones((25,25),np.float32)/(25**2)
        ratio = 100
        kernel_width = w//ratio
        kernel_height = h//ratio
        kernel = np.ones((kernel_width, kernel_height), np.float32)/(kernel_width * kernel_height)
        blurred_image = cv2.filter2D(depth_image,-1,kernel)
        cv2.imwrite('convolved image.png', dst)
        return blurred_image
        

    def get_position_of_obstacle(depth_image):
        #algorithm to check for nearest obstacle
        grid_size = [4, 4]
        blurred_depth_image = blurred_depth_image(depth_image, h, w)
        h, w = depth_image.shape
        cell_h, cell_w = h // grid_size[0], w // grid_size[1]




    def publish_depth_image(self):
        try:
            depth_image = self.get_depth_image_array()

            center_point_depth = self.get_center_point(depth_image)

            # Create a ROS Image message using cv_bridge
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='16UC1')
            self.publisher.publish(depth_msg)
            
            # self.get_logger().info("Published depth image. Depth @ center: " + str(center_point_depth) + 'mm')

            self.get_logger().info("Published depth image. Center Point Depth: " + str(center_point_depth))
            # self.get_logger().info(f"Depth Averages Grid:\n{depth_averages}")
        except Exception as e:
            self.get_logger().error(f"Error publishing depth image: {e}")


    def publish_nearest_coordinates(self):
        depth_image = self.get_depth_image_array()

        #create a message for sending the nearest coordinates
        data = None
        self.publisher.publish(data)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def read_in_image(file_path):
    #mg = cv2.imread('/Users/sara/Pictures/10-10-6k.jpg')
    #assert img is not None, "file could not be read, check with os.path.exists()"
    img = cv2.imread(file_path)
    assert img is not None
    return img

def test(args=None):
    image = read_in_image()
    rclpy.init(args=args)


if __name__ == '__main__':
    #main()
    test()
 
