import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

class DepthListenerNode(Node):
    def __init__(self):                                               
        super().__init__('depth_listener_node')

        # Add frame counter and interval parameters
        self.frame_count = 0
        self.frame_interval = 1  # Process every 30th frame
        # Create a subscriber for the depth topic
        self.subscription = self.create_subscription(
            Image,               # Message type
            '/camera/camera/depth/image_rect_raw',  # Topic name (match this to your topic)
            self.depth_image_callback, # Callback function when a message is received
            10)                  # QoS settings (10 means at least 10 messages are buffered)
        # self.test()

        #self.subscription  # Prevent unused variable warning

        # Create a CvBridge to convert ROS Image messages to OpenCV format
        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(Float32MultiArray,'position_data', 10)

        # self.timer = self.create_subscri(0.5, self.depth_image_callback)
       

    def depth_image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.frame_interval != 0:
           return
        try:
            
            # Convert ROS Image message to OpenCV image
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')        
            # Display depth image
            self.display_depth_image(depth_image)
       
            #return depth_image_meters
            self.publish_direction_msg(depth_image)
            ##### RUN ANY FOLLOWUP FUNCTIONS INSIDE OF THE CALLBACK #####
        except Exception as e:
            self.get_logger().error(f"Error converting ROS Image to OpenCV: {e}")

    def display_depth_image(self, depth_image):
        # Normalize depth image to 8-bit for display purposes
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
        # Convert to 8-bit image for display (optional)
        depth_image_normalized = np.uint8(np.asanyarray(depth_image_normalized))
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

    def blur_depth_image(self,depth_image, h, w):
        #kernel = np.ones((25,25),np.float32)/(25**2)
        ratio = 100
        kernel_width = w//ratio
        kernel_height = h//ratio
        kernel = np.ones((kernel_width, kernel_height), np.float32)/(kernel_width * kernel_height)
        blurred_image = cv2.filter2D(depth_image,-1,kernel)
        # cv2.imwrite('convolved_image.png', blurred_image)
        return blurred_image

    def get_position_distance_of_obstacle(self,depth_image, grid_size):
        #algorithm to check for nearest obstacle
        #grid_size = [16, 16] #row size, col size
        h, w = depth_image.shape
        blurred_depth_image = self.blur_depth_image(depth_image, h, w)
        cell_h, cell_w = h // grid_size[0], w // grid_size[1]
        blurred_depth_image_averages = np.zeros(grid_size)

        for i in range(grid_size[0]):
            for j in range(grid_size[1]):
                cell = blurred_depth_image[(i) * cell_h: (i + 1) * cell_h, (j) * cell_w: (j + 1) * cell_w]
                cell_average = np.mean(cell)
                blurred_depth_image_averages[i, j] = cell_average

        min_value_position = np.argmin(blurred_depth_image_averages)
        min_value_distance = np.min(blurred_depth_image_averages)
        #unravaled_max_value_position = np.unravel_index(max_value_position, np.array(grid_size).shape)
        unraveled_x_position = min_value_position // grid_size[0]
        unraveled_y_position = min_value_position % grid_size[1]
        unraveled_min_value_position = np.array([unraveled_x_position, unraveled_y_position])
        print(unraveled_min_value_position.shape)
        return unraveled_min_value_position, min_value_distance

    def publish_direction_msg(self, depth_image, grid_size=[12, 12], distance_threshold=500):
        position, distance = self.get_position_distance_of_obstacle(depth_image, grid_size)
        lmr_position = int(position[0] // (grid_size[0]/3))
        print(lmr_position)
        output_value = [float(-1.0), float(-1.0), float(-1.0)]

        if(distance < distance_threshold):
            output_value[lmr_position] = float(distance)

        # Creating a Float32MultiArray message
        msg = Float32MultiArray()

        # Define the layout (e.g., a 2x3 matrix)
        layout = MultiArrayLayout()

        # Dimensions
        dim1 = MultiArrayDimension()
        dim1.label = "rows"
        dim1.size = 1
        dim1.stride = 3  # Number of columns

        dim2 = MultiArrayDimension()
        dim2.label = "cols"
        dim2.size = 3
        dim2.stride = 1

        layout.dim = [dim1, dim2]
        layout.data_offset = 0

        msg.layout = layout

        # Populate the data (flattened row-major order)
        msg.data = output_value
        
        self.publisher_.publish(msg)
        # print("Publishing Data")
        #return output_msg #returns ros Vector3 message for publisher


    def read_in_image(self, file_path):
        #mg = cv2.imread('/Users/sara/Pictures/10-10-6k.jpg')
        #assert img is not None, "file could not be read, check with os.path.exists()"
        img = cv2.imread(file_path)
        assert img is not None
        print(img.shape)
        return img

    def test(self):
        # file_path = "/home/robert27/x1robot/BruinBear/image_convolution/realsense_depth_image_Depth.png"
        # image = self.read_in_image(file_path)
        position_of_obstacle = self.get_position_of_obstacle(self.depth_image_callback)
        
        print(position_of_obstacle)


def main(args=None):
    rclpy.init(args=args)
    listener_node = DepthListenerNode()  # Create the node
    #publisher_node = DepthPublisherNode()
    # node.destroy_node()  # Ensure node is destroyed properly
    # rclpy.shutdown()  # Ensure proper shutdown of ROS2 context
    
    try:
        #rclpy.spin(listener_node)  # Spin the node to keep it alive
        rclpy.spin(listener_node)
    
    
        pass
    finally:
        # Shutdown the node gracefully on exit
        rclpy.shutdown()
        pass


if __name__ == '__main__':
    main()
