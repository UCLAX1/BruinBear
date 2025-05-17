import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import numpy as np
import cv2
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3

class RealSenseNode(Node):
    def __init__(self):
        super().__init__('realsense_node')
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Vector3, 'vector', 10)

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
        cv2.imshow('depth', depth_image)
        return depth_image

    #get stream of images and continously publish
    def get_flow_of_images(self):
        while True:
            self.get_depth_image_array()
            if cv2.waitKey(1) == ord('q'):
                self.pipe.stop()
                self.get_depth_image_array()
           

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
        obstacle_position = get_position_of_obstacle(depth_image)
        
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

def get_center_point(depth_image):
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

    center_point_depth= depth_image[depth_image.shape[0]//2, depth_image.shape[1]//2]

    return center_point_depth


def blur_depth_image(depth_image, h, w):
    #kernel = np.ones((25,25),np.float32)/(25**2)
    ratio = 100
    kernel_width = w//ratio
    kernel_height = h//ratio
    kernel = np.ones((kernel_width, kernel_height), np.float32)/(kernel_width * kernel_height)
    blurred_image = cv2.filter2D(depth_image,-1,kernel)
    #cv2.imwrite('convolved image.png', blurred_image)
    cv2.imwrite('convolved_image.png', blurred_image)
    return blurred_image


def get_position_of_obstacle(depth_image):
    #algorithm to check for nearest obstacle
    grid_size = [16, 16] #row size, col size
    h, w, c = depth_image.shape
    blurred_depth_image = blur_depth_image(depth_image, h, w)
    cell_h, cell_w = h // grid_size[0], w // grid_size[1]
    blurred_depth_image_averages = np.zeros(grid_size)

    for i in range(grid_size[0]):
        for j in range(grid_size[1]):
            cell = blurred_depth_image[(i) * cell_h: (i + 1) * cell_h, (j) * cell_w: (j + 1) * cell_w]
            cell_average = np.mean(cell)
            blurred_depth_image_averages[i, j] = cell_average

    min_value_position = np.argmin(blurred_depth_image_averages)
    min_value = np.min(blurred_depth_image_averages)
    #unravaled_max_value_position = np.unravel_index(max_value_position, np.array(grid_size).shape)
    unraveled_x_position = min_value_position // grid_size[0]
    unraveled_y_position = min_value_position % grid_size[1]
    unraveled_max_value_position = np.array([unraveled_x_position, unraveled_y_position])
    print(unraveled_max_value_position.shape)
    return unraveled_max_value_position

def determine_movement(min_value_distance, min_value_position):
    threshold_distance = 1
    if(min_value_distance < threshold_distance):
        twist_msg = Twist()
        self.publisher.publish(twist_msg)
    return 

def read_in_image(file_path):
    #mg = cv2.imread('/Users/sara/Pictures/10-10-6k.jpg')
    #assert img is not None, "file could not be read, check with os.path.exists()"
    img = cv2.imread(file_path)
    assert img is not None
    print(img.shape)
    return img

def test(args=None):
    file_path = "/home/robert27/x1robot/BruinBear/image_convolution/realsense_depth_image_Depth.png"
    image = read_in_image(file_path)
    position_of_obstacle = get_position_of_obstacle(image)
    
    print(position_of_obstacle)
    
    
if __name__ == '__main__':
    #main()
    #/home/robert27/x1robot/BruinBear/ros2_ws/src/sensing/sensing/realsense.py
    test()
 
