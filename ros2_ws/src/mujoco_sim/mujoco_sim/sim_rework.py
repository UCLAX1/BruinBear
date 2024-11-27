import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

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

modelPath = 'ros2_ws/Quadroped-XML/quadroped_og.xml'
displayRefreshRate = 120

class ActuatorPositionPub(Node):
  def __init__(self):
    super().__init__('actuator_pos_pub')
    self.publisher_ = self.create_publisher(String, 'actuator_postions', 10)
    timer_period = 1
    self.timer = self.create_timer(timer_period, self.pub_actuator_pos)
  
  def pub_actuator_pos(self, actuator_positions):
    msg = String()
    msg.data = str(actuator_positions)
    self.publisher_.publish(msg)
    print("\033c") # disable if this causes problems, just clears the terminal
    self.get_logger().info('Actuator Positions: "%s"' % msg.data)


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

    # determine action based on mouse button
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

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Quadraped", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
cam.azimuth = 45
cam.elevation = -35
cam.distance = 2
cam.lookat = np.array([0.0, 0.0, 0])

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

rclpy.init()
actuatorPositionNode = ActuatorPositionPub()
counter = 0
globalRobotState = 'start'
globalHipStatus = 'not moving'
globalKneeStatus = 'not moving'
startedWalking = False
motorSpeed = 0.0009



def calcActRotation(tx, ty, backLeg):
    r1 = 0.177
    r2 = 0.177
    # The shoulder is at the origin and the target position is defined as being in the 3rd or 4th quadrants
    tx = tx
    ty = ty
    dT = np.sqrt(tx**2 + ty**2)
    th1 = np.arccos((r1**2 + r2**2 - dT**2)/(2 * r1 * r2))
    th2 = np.arccos((r1**2 + dT**2 - r2**2)/(2 * r1 * dT))
    if tx == 0:
        th3 = np.pi/2 + th2
    else:
        th3 = np.arctan(ty/tx) + th2
    # if abs(th3) < 0.0001:
    #     th3 = np.pi

    if tx < 0:
        th3 += np.deg2rad(90)
    else:
        th3 -= np.deg2rad(90)
    # if backLeg:
    #     th1 += np.deg2rad(202)
    #     # th1 += (180 - th3)
    #     # th3 *= 1.2
    # # th1 = np.pi - th1
    if th3 <= (np.pi/2):
        if th3 > 0:
            th3 *= -1
    if backLeg:
        th3 *= -1
        th1 -= np.pi
        # th1 *= 2
        # th1 -= (th3 - (np.pi/2))
        print(th3)
        print(th1)
    else:
        th1 = np.pi - th1
    return th1, th3

def moveLeg(actuator, position, actType, backLeg):
    global globalHipStatus
    global globalKneeStatus
    xPos = position[0]
    yPos = position[1]

    hipRotTarg = calcActRotation(xPos,yPos, backLeg)[1]
    kneeRotTarg = calcActRotation(xPos,yPos, backLeg)[0]
    
    if actType == 'hip':
        if hipRotTarg < data.ctrl[actuator]:
            data.ctrl[actuator] -= motorSpeed
            globalHipStatus = 'moving'
        else:
            data.ctrl[actuator] += motorSpeed
            globalHipStatus = 'moving'

        if data.ctrl[actuator] >= hipRotTarg - 0.001 and data.ctrl[actuator] <= hipRotTarg + 0.001:
            globalHipStatus = 'not moving'

    if actType == 'knee':
        if kneeRotTarg < data.ctrl[actuator]:
            data.ctrl[actuator] -= motorSpeed
            globalKneeStatus = 'moving'
        else:
            data.ctrl[actuator] += motorSpeed
            globalKneeStatus = 'moving'

        if data.ctrl[actuator] >= kneeRotTarg - 0.001 and data.ctrl[actuator] <= kneeRotTarg + 0.001:
            globalKneeStatus = 'not moving'
    

def layDown():
    global globalRobotState
    global globalHipStatus
    if globalRobotState == 'laying down':
        positionTargetX = 0
        positionTargetY = 0.05
        moveLeg(FLH, [positionTargetX,positionTargetY],'hip')
        moveLeg(FLK, [positionTargetX,positionTargetY],'knee')

        moveLeg(FRH, [positionTargetX,positionTargetY],'hip')
        moveLeg(FRK, [positionTargetX,positionTargetY],'knee')

        moveLeg(BLH, [positionTargetX,positionTargetY],'hip')
        moveLeg(BLK, [positionTargetX,positionTargetY],'knee')

        moveLeg(BRH, [positionTargetX,positionTargetY],'hip')
        moveLeg(BRK, [positionTargetX,positionTargetY],'knee')

        if globalHipStatus == 'not moving' and globalKneeStatus == 'not moving':
            globalRobotState = 'finished'
    elif globalRobotState == 'finished' or globalRobotState == 'start':
        globalRobotState = 'laying down'

def neutralPos():
    global globalRobotState
    global globalHipStatus
    if globalRobotState == 'neutral pos':
        positionTargetX = 0.06
        positionTargetY = 0.3
        moveLeg(FLH, [positionTargetX,positionTargetY],'hip', False)
        moveLeg(FLK, [positionTargetX,positionTargetY],'knee', False)

        moveLeg(FRH, [positionTargetX,positionTargetY],'hip', False)
        moveLeg(FRK, [positionTargetX,positionTargetY],'knee', False)

        moveLeg(BLH, [-positionTargetX,positionTargetY],'hip', True)
        moveLeg(BLK, [-positionTargetX,positionTargetY],'knee', True)

        moveLeg(BRH, [-positionTargetX,positionTargetY],'hip', True)
        moveLeg(BRK, [-positionTargetX,positionTargetY],'knee', True)
        if globalHipStatus == 'not moving' and globalKneeStatus =='not moving':
            globalRobotState = 'finished'
    elif globalRobotState == 'finished' or globalRobotState == 'start':
        globalRobotState = 'neutral pos'
def pushUp():
    layDown()
    neutralPos()


walkCounter = 0
walkingPosX = 0.02
walkingLiftPosX = walkingPosX/2
walkingPosY = 0.3
walkingLiftPosY = 0.29
rearOffset = 0.0
class RobotStateMachine:
    walkCounter = 0
    def __init__(self):
        self.state = 'get neutral'
        self.direction = 1  # Set to 1 for forward, -1 for backward

    def step(self):
        # print(self.state)
        if (self.state == 'get neutral'):
            self.neutralPos()
        elif self.state == 'INIT':
            self.FL_BR_Drop(-self.direction)
            self.FR_BL_Lift(self.direction)
        elif self.state == 'FR_BL Lifted':
            self.FR_BL_Drop(self.direction)
        elif self.state == 'FR_BL Dropped':
            self.FR_BL_Drop(-self.direction)
            self.FL_BR_Lift(self.direction)
        elif self.state == 'FL_BR Lifted':
            self.FL_BR_Drop(self.direction)
        elif self.state == 'FL_BR Dropped':
            # print("Sequence complete: Resetting to INIT")
            self.walkCounter += 1
            self.state = 'INIT'  # Reset or set up for next step

    def neutralPos(self):
        positionTargetX = 0.08
        positionTargetY = 0.3
        moveLeg(FLH, [positionTargetX,positionTargetY],'hip', False)
        moveLeg(FLK, [positionTargetX,positionTargetY],'knee', False)

        moveLeg(FRH, [positionTargetX,positionTargetY],'hip', False)
        moveLeg(FRK, [positionTargetX,positionTargetY],'knee', False)

        moveLeg(BLH, [positionTargetX *2,positionTargetY],'hip', True)
        moveLeg(BLK, [positionTargetX *2,positionTargetY],'knee', True)

        moveLeg(BRH, [positionTargetX *2,positionTargetY],'hip', True)
        moveLeg(BRK, [positionTargetX *2,positionTargetY],'knee', True)
        if self.check_position(FRK, 'knee', positionTargetX, positionTargetY, False) \
                and self.check_position(BRK, 'knee', positionTargetX *2, positionTargetY, True):
            self.state = 'INIT'


    def FR_BL_Lift(self, d):
        global walkingLiftPosX, walkingLiftPosY, rearOffset
        moveLeg(FRK, [walkingLiftPosX * d, walkingLiftPosY], 'knee', False)
        moveLeg(FRH, [walkingLiftPosX * d, walkingLiftPosY], 'hip', False)
        moveLeg(BLK, [walkingLiftPosX * -d, walkingLiftPosY+ rearOffset], 'knee', True)
        moveLeg(BLH, [walkingLiftPosX * -d, walkingLiftPosY+ rearOffset], 'hip', True)
        
        if self.check_position(FRK, 'knee', walkingLiftPosX * d, walkingLiftPosY, False) \
                and self.check_position(BLK, 'knee', walkingLiftPosX * -d, walkingLiftPosY+ rearOffset, True):
            self.state = 'FR_BL Lifted'

    def FR_BL_Drop(self, d):
        global walkingPosX, walkingPosY, rearOffset
        moveLeg(FRK, [walkingPosX * d, walkingPosY], 'knee', False)
        moveLeg(FRH, [walkingPosX * d, walkingPosY], 'hip', False)
        moveLeg(BLK, [walkingPosX * -d, walkingPosY+ rearOffset], 'knee', True)
        moveLeg(BLH, [walkingPosX * -d, walkingPosY+ rearOffset], 'hip', True)
        
        if self.check_position(FRK, 'knee', walkingPosX * d, walkingPosY, False) \
                and self.check_position(BLK, 'knee', walkingPosX * -d, walkingPosY+ rearOffset, True):
            self.state = 'FR_BL Dropped'


    def FL_BR_Lift(self, d):
        global walkingLiftPosX, walkingLiftPosY, rearOffset
        moveLeg(FLK, [walkingLiftPosX * d, walkingLiftPosY], 'knee', False)
        moveLeg(FLH, [walkingLiftPosX * d, walkingLiftPosY], 'hip', False)
        moveLeg(BRK, [walkingLiftPosX * -d, walkingLiftPosY+ rearOffset], 'knee', True)
        moveLeg(BRH, [walkingLiftPosX * -d, walkingLiftPosY+ rearOffset], 'hip', True)
        
        if self.check_position(FLK, 'knee', walkingLiftPosX * d, walkingLiftPosY, False) \
                and self.check_position(BRK, 'knee', walkingLiftPosX * -d, walkingLiftPosY + rearOffset, True):
            self.state = 'FL_BR Lifted'

    def FL_BR_Drop(self, d):
        # global walkingPosStatus
        global walkingPosX, walkingPosY, rearOffset
        # if walkingPosStatus == 'FL BR Lifted' or walkingPosStatus == 'Dropping FL BR' or walkingPosStatus == 'none':
            # walkingPosStatus = 'Dropping FL BR'
        moveLeg(FLK, [walkingPosX * d, walkingPosY], 'knee', False)
        moveLeg(FLH, [walkingPosX * d, walkingPosY], 'hip', False)
        moveLeg(BRK, [walkingPosX * -d, walkingPosY+ rearOffset], 'knee', True)
        moveLeg(BRH, [walkingPosX * -d, walkingPosY+ rearOffset], 'hip', True)
        
        if self.check_position(FLK, 'knee', walkingPosX * d, walkingPosY, False) \
                and self.check_position(BRK, 'knee', walkingPosX * -d, walkingPosY + rearOffset, True):
            # walkingPosStatus = 'FL BR Dropped'
            self.state = 'FL_BR Dropped'


    def check_position(self, actIndex, actType, posX, posY, backLeg):
        global targetPosPub
        global currentPosPub
        resultIndex = 0
        if actType == 'hip':
            resultIndex = 1
        targetPos = calcActRotation(posX, posY, backLeg)[resultIndex]
        targetPosPub = targetPos
        currentPos = data.ctrl[actIndex]
        currentPosPub = data.ctrl
        if currentPos >= targetPos - 0.001 and currentPos <= targetPos + 0.001:
            return True
        return False

robot_fsm = RobotStateMachine()



lookPosX = 0
lookPosY = 0

# robotFSM = FiniteStateMachine()
while not glfw.window_should_close(window):
    time_prev = data.time
    direction = 0
    while data.time - time_prev < 1.0/displayRefreshRate:

        counter += 1
        # robotFSM.step()
        robot_fsm.step()
        
        if counter % 100 == 0:
            pass
            # temp = ''
            # hipTarget = str(calcActRotation(positionTargetX,positionTargetY)[1])
            # kneeTarget = str(calcActRotation(positionTargetX,positionTargetY)[0])
            # actuatorPositionNode.pub_actuator_pos()
            # actuatorPositionNode.pub_actuator_pos(str(data.ctrl[FRH]) + ' ' + globalHipStatus + ' ' + globalRobotState + ' ' + str(walkingPosXFwd) + ' ' + str(walkCounter))
       
        mj.mj_step(model, data)


    # if (data.time>=simend):
    #     break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    # if (print_camera_config==1):
    #     print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
    #     print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')
    smoothing_factor = 0.1
    robot_x = data.xpos[1][0]  # X-coordinate of the robot
    robot_y = data.xpos[1][1]  # Y-coordinate of the robot 
    cam.lookat = [robot_x, robot_y, 0.2]

    cam.distance = 2  # Adjust this distance as needed
    cam.azimuth = 45   # Keep or modify this for different angles
    cam.elevation = -35 # Adjust the elevation if necessary
    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()