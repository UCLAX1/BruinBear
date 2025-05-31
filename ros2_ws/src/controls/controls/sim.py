import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from controls.foot_traj_follower import startPos
import numpy as np
from scipy.spatial.transform import Rotation as R

# xml_path = 'hello.xml' #xml file (assumes this is in the same folder as this file)
simend = 10 #simulation time
print_camera_config = 1 #set to 1 to print camera config
                        #this is useful for initializing view of the model

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

modelPath = 'quadruped-new/quadruped.xml'
displayRefreshRate = 30
joints = startPos

class jointPosSub(Node):

    def __init__(self):
        super().__init__('joint_pos_sub')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'imu_data', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'joint_positions',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Float32MultiArray, 'rangefinder_data', 10)

    def listener_callback(self, msg):
        # self.get_logger().info('recieved positions')
        global joints 
        joints = msg.data

    def pub_imu_data(self, imu_data):
        msg = Float32MultiArray()
        msg.data = imu_data
        #msg.data = [0.0,0.0]
        self.publisher_.publish(msg)
        #self.get_logger().info(f'Published imu data: {msg.data}')
    # def pub_rangefinder_data(self):
    #     msg = Float32MultiArray()
    #     if(getRange('range1') < 2000):
    #         msg.data.append(getRange('range2'))
    #     else:
    #         msg.data.append(-1)
    #     if(getRange('range2') < 2000):
    #         msg.data.append(getRange('range1'))
    #     else:
    #         msg.data.append(-1)
    #     if(getRange('range3') < 2000):
    #         msg.data.append(getRange('range3'))
    #     else:
    #         msg.data.append(-1)
    #     self.publisher_.publish(msg)
        #self.get_logger().info("publishing: " + str(msg)) 

def get_yaw_from_quaternion(quaternion):
    """
    Extracts the yaw angle (rotation around the z-axis) from a given quaternion.
    
    :param quaternion: A list or array of [x, y, z, w] representing the quaternion.
    :return: Yaw angle in radians.
    """
    # Convert quaternion to rotation object
    r = R.from_quat(quaternion)  # SciPy expects [x, y, z, w]
    
    # Convert to Euler angles (returns in radians)
    euler_angles = r.as_euler('xyz', degrees=False)
    
    # Extract the yaw (rotation around z-axis)
    yaw = euler_angles[0]  
    return yaw

keyBoardControl = 'none'
cubeControl1 = 'none'
cubeControl2 = 'none'
cubeControl3 = 'none'
cameraControl = 'none'

def keyboard_callback(window, key, scancode, action, mods):
    global keyBoardControl, cameraControl, cubeControl1, cubeControl2, cubeControl3
    if action == glfw.PRESS or action == glfw.REPEAT:  # Handle key press or hold
        if key == glfw.KEY_W:  # Move forward)
            keyBoardControl = 'fwd'
        elif key == glfw.KEY_A:  # Turn left
            keyBoardControl = 'lft'
        elif key == glfw.KEY_D:  # Turn right
            keyBoardControl = 'rgt'
        elif key == glfw.KEY_S:
            keyBoardControl = 'bck'
        elif key == glfw.KEY_X:  # Neutral position or stop
            keyBoardControl = 'nut'
        elif key == glfw.KEY_UP:
            cameraControl = 'up'
        elif key == glfw.KEY_DOWN:
            cameraControl = 'down'
        elif key == glfw.KEY_RIGHT:
            cameraControl = 'right'
        elif key == glfw.KEY_LEFT:
            cameraControl = 'left'
        elif key == glfw.KEY_DELETE:
            cameraControl = 'home'

        if key == glfw.KEY_G:
            cubeControl1 = 'cube_fwd'
        elif key == glfw.KEY_V:
            cubeControl1 = 'cube_bck'
        elif key == glfw.KEY_C:
            cubeControl1 = 'cube_left'
        elif key == glfw.KEY_B:
            cubeControl1 = 'cube_right'
        elif key == glfw.KEY_N:
            cubeControl1 = 'cube_rot_ccw'
        elif key == glfw.KEY_M:
            cubeControl1 = 'cube_rot_cw'

        if key == glfw.KEY_I:
            cubeControl2 = 'cube_fwd'
        elif key == glfw.KEY_K:
            cubeControl2 = 'cube_bck'
        elif key == glfw.KEY_L:
            cubeControl2 = 'cube_left'
        elif key == glfw.KEY_J:
            cubeControl2 = 'cube_right'
        elif key == glfw.KEY_U:
            cubeControl2 = 'cube_rot_ccw'
        elif key == glfw.KEY_O:
            cubeControl2 = 'cube_rot_cw'

        if key == glfw.KEY_SEMICOLON:
            cubeControl3 = 'cube_fwd'
        elif key == glfw.KEY_PERIOD:
            cubeControl3 = 'cube_bck'
        elif key == glfw.KEY_COMMA:
            cubeControl3 = 'cube_left'
        elif key == glfw.KEY_SLASH:
            cubeControl3 = 'cube_right'
        elif key == glfw.KEY_Q:
            cubeControl3 = 'cube_rot_ccw'
        elif key == glfw.KEY_P:
            cubeControl3 = 'cube_rot_cw'
        
    elif action == glfw.RELEASE:
        if key == glfw.KEY_W:  # Move forward
            keyBoardControl = 'nut'
        elif key == glfw.KEY_A:  # Turn left
            keyBoardControl = 'nut'
        elif key == glfw.KEY_D:  # Turn right
            keyBoardControl = 'nut'
        elif key == glfw.KEY_S:
            keyBoardControl = 'nut'
        elif key == glfw.KEY_X:  # Neutral position or stop
            keyBoardControl = 'nut'
        
        if key == glfw.KEY_UP:
            cameraControl = 'none'
        elif key == glfw.KEY_DOWN:
            cameraControl = 'none'
        elif key == glfw.KEY_RIGHT:
            cameraControl = 'none'
        elif key == glfw.KEY_LEFT:
            cameraControl = 'none'
        elif key == glfw.KEY_DELETE:
            cameraControl = 'none'

        if key == glfw.KEY_G:
            cubeControl1 = 'none'
        elif key == glfw.KEY_V:
            cubeControl1 = 'none'
        elif key == glfw.KEY_C:
            cubeControl1 = 'none'
        elif key == glfw.KEY_B:
            cubeControl1 = 'none'
        elif key == glfw.KEY_N:
            cubeControl1 = 'none'
        elif key == glfw.KEY_M:
            cubeControl1 = 'none'
        
        if key == glfw.KEY_I:
            cubeControl2 = 'none'
        elif key == glfw.KEY_K:
            cubeControl2 = 'none'
        elif key == glfw.KEY_J:
            cubeControl2 = 'none'
        elif key == glfw.KEY_L:
            cubeControl2 = 'none'
        elif key == glfw.KEY_U:
            cubeControl2 = 'none'
        elif key == glfw.KEY_O:
            cubeControl2 = 'none'

        if key == glfw.KEY_SEMICOLON:
            cubeControl3 = 'none'
        elif key == glfw.KEY_PERIOD:
            cubeControl3 = 'none'
        elif key == glfw.KEY_COMMA:
            cubeControl3 = 'none'
        elif key == glfw.KEY_SLASH:
            cubeControl3 = 'none'
        elif key == glfw.KEY_Q:
            cubeControl3 = 'none'
        elif key == glfw.KEY_P:
            cubeControl3 = 'none'


def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass

def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based son mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
# dirname = os.path.dirname(__file__)
# abspath = os.path.join(dirname + "/" + xml_path)
# xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(modelPath)  # MuJoCo model
data = mj.MjData(model)                     # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

cube1 = mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT, 'box-joint1')
cube_qpos_addr1 = model.jnt_qposadr[cube1]  # Get qpos index for the cube joint

cube2 = mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT, 'box-joint2')
cube_qpos_addr2 = model.jnt_qposadr[cube2]  # Get qpos index for the cube joint

cube3 = mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT, 'box-joint3')
cube_qpos_addr3 = model.jnt_qposadr[cube3]  # Get qpos index for the cube joint

bear = mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT, 'wrapper')
bear_qpos = model.jnt_qposadr[cube3]  # Get qpos index for bear

com_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'com-sphere')

# def getRange(rangefinder_name):
#     # mujoco.mj_resetData(model, data)
#     # data.ctrl = 20
#     return data.sensor(rangefinder_name).data.copy()

rclpy.init()
counter = 0
globalRobotState = 'start'
startedWalking = False
motorSpeed = 0.001


def main(args=None):
    # Init GLFW, create window, make OpenGL context current, request v-sync
    glfw.init()
    window = glfw.create_window(1200, 900, "Demo", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)

    # initialize visualization data structures
    mj.mjv_defaultCamera(cam)
    mj.mjv_defaultOption(opt)
    scene = mj.MjvScene(model, maxgeom=10000)
    context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

    # install GLFW mouse and keyboard callbacks
    glfw.set_key_callback(window, keyboard_callback)
    glfw.set_cursor_pos_callback(window, mouse_move)
    glfw.set_mouse_button_callback(window, mouse_button)
    glfw.set_scroll_callback(window, scroll)

    #initialize the controller
    init_controller(model,data)

    #set the controller
    mj.set_mjcb_control(controller)
    FRH = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'FR-H-servo')
    FLH = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'FL-H-servo')
    BRH = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'BR-H-servo')
    BLH = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'BL-H-servo')

    FRK = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'FR-K-servo')
    FLK = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'FL-K-servo')
    BRK = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'BR-K-servo')
    BLK = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'BL-K-servo')

    FRR = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'FR-R-servo')
    FLR = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'FL-R-servo')
    BRR = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'BR-R-servo')
    BLR = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'BL-R-servo')

    #rclpy.init()
    simNode = jointPosSub()
    counter = 0
    global joints

    cam.distance = 2  # Adjust this distance as needed
    cam.azimuth = 45   # Keep or modify this for different angles
    cam.elevation = -35 # Adjust the elevation if necessary
    # Update scene and render

    # simNode.get_logger().info(f'Published imu data: {data.qpos}')
    # simNode.get_logger().info(f'Published imu data: {data.qpos[cube_qpos_addr2]}')
    # simNode.get_logger().info(f'Published imu data: {cube_qpos_addr2}')

    while not glfw.window_should_close(window):
        time_prev = data.time

        while data.time - time_prev < 1.0/displayRefreshRate:
            rclpy.spin_once(simNode, timeout_sec=0)
            # simNode.pub_rangefinder_data()
            data.ctrl[FLH] = joints[0]
            data.ctrl[FRH] = joints[1]
            data.ctrl[BRH] = joints[2]
            data.ctrl[BLH] = joints[3]
            data.ctrl[FLK] = joints[4]
            data.ctrl[FRK] = joints[5]
            data.ctrl[BRK] = joints[6]
            data.ctrl[BLK] = joints[7]
            data.ctrl[FLR] = joints[8]
            data.ctrl[FRR] = joints[9]
            data.ctrl[BRR] = joints[10]
            data.ctrl[BLR] = joints[11]
            
            if cubeControl1 == 'cube_fwd':
                data.qpos[cube_qpos_addr1] += 0.001 
            elif cubeControl1 == 'cube_bck':
                data.qpos[cube_qpos_addr1] -= 0.001
            elif cubeControl1 == 'cube_right':
                data.qpos[cube_qpos_addr1 + 1] += 0.001
            elif cubeControl1 == 'cube_left':
                data.qpos[cube_qpos_addr1 + 1] -= 0.001
            elif cubeControl1 == 'cube_rot_cw':
                data.qpos[cube_qpos_addr1 + 6] += 0.001
            elif cubeControl1 == 'cube_rot_ccw':
                data.qpos[cube_qpos_addr1 + 6] -= 0.001
            
            if cubeControl2 == 'cube_fwd':
                data.qpos[cube_qpos_addr2] += 0.001 
            elif cubeControl2 == 'cube_bck':
                data.qpos[cube_qpos_addr2] -= 0.001
            elif cubeControl2 == 'cube_right':
                data.qpos[cube_qpos_addr2 + 1] += 0.001
            elif cubeControl2 == 'cube_left':
                data.qpos[cube_qpos_addr2 + 1] -= 0.001
            elif cubeControl2 == 'cube_rot_cw':
                data.qpos[cube_qpos_addr2 + 6] += 0.001
            elif cubeControl2 == 'cube_rot_ccw':
                data.qpos[cube_qpos_addr2 + 6] -= 0.001

            if cubeControl3 == 'cube_fwd':
                data.qpos[cube_qpos_addr3] += 0.001 
            elif cubeControl3 == 'cube_bck':
                data.qpos[cube_qpos_addr3] -= 0.001
            elif cubeControl3 == 'cube_right':
                data.qpos[cube_qpos_addr3 + 1] += 0.001
            elif cubeControl3 == 'cube_left':
                data.qpos[cube_qpos_addr3 + 1] -= 0.001
            elif cubeControl3 == 'cube_rot_cw':
                data.qpos[cube_qpos_addr3 + 6] += 0.001
            elif cubeControl3 == 'cube_rot_ccw':
                data.qpos[cube_qpos_addr3 + 6] -= 0.001

            if cameraControl == 'down':
                cam.elevation -= 0.1
            elif cameraControl == 'up':
                cam.elevation += 0.1
            elif cameraControl == 'right':
                cam.azimuth -= 0.1
            elif cameraControl == 'left':
                cam.azimuth += 0.1
            elif cameraControl == 'home':
                if cam.azimuth > 45:
                    while cam.azimuth > 45:
                        cam.azimuth -= 0.1
                if cam.azimuth < 45:
                    while cam.azimuth < 45:
                        cam.azimuth += 0.1
                if cam.elevation > -35:
                    while cam.elevation > -35:
                        cam.elevation -= 0.1
                if cam.elevation < -35:
                    while cam.elevation < -35:
                        cam.elevation += 0.1
            

            mj.mj_step(model, data)
            if (counter % 100 == 0 ):
                continue
                #simNode.get_logger().info(f'Published imu data: {data.qpos}')
                #simNode.pub_imu_data([round(data.qpos[bear_qpos][0],4),round(data.qpos[bear_qpos][1],4),round(data.qpos[bear_qpos][2],4),float(round(get_yaw_from_quaternion(data.qpos[bear_qpos][3:7]),4))])
            counter += 1

        viewport_width, viewport_height = glfw.get_framebuffer_size(
            window)
        viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

        smoothing_factor = 0.1
        robot_x = data.xpos[1][0]  # X-coordinate of the robot
        robot_y = data.xpos[1][1]  # Y-coordinate of the robot 
        cam.lookat = [robot_x, robot_y, 0.2]

        mj.mjv_updateScene(model, data, opt, None, cam,
                        mj.mjtCatBit.mjCAT_ALL.value, scene)
        mj.mjr_render(viewport, scene, context)

        # swap OpenGL buffers (blocking call due to v-sync)
        glfw.swap_buffers(window)

        # process pending GUI events, call GLFW callbacks
        glfw.poll_events()

    glfw.terminate()
    rclpy.shutdown()