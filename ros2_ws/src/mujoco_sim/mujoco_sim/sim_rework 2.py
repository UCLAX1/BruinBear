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

modelPath = 'ros2_ws/quadruped-new/quadruped.xml'
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

FRR = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'FR-R-servo')
FLR = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'FL-R-servo')
BRR = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'BR-R-servo')
BLR = mj.mj_name2id(model, mj.mjtObj.mjOBJ_ACTUATOR, 'BL-R-servo')

rclpy.init()
actuatorPositionNode = ActuatorPositionPub()
counter = 0
globalRobotState = 'start'
startedWalking = False
motorSpeed = 0.001


def calcActRotation(tx, ty, tz, backleg):
    r1 = .180
    r2 = .199678
    tx *= -1

    dT = np.sqrt(tx**2 + ty**2 + tz**2)
    kneeHipPaw = np.arccos((r1**2 + dT**2 - r2**2)/(2 * r1 * dT))
    pawKneeHip = np.arccos((r1**2 + r2**2 - dT**2)/(2 * r1 * r2))
    print('pawKneeHip', np.rad2deg(pawKneeHip))
    print('kneeHipPaw', np.rad2deg(kneeHipPaw))


    hip = np.arccos(-tx/dT) - kneeHipPaw
    print(np.arctan(-tx/(ty)))
    print((np.arccos((-tx)/dT) - np.pi/2) * -1)

    knee = np.pi - pawKneeHip

    if backleg:
        hip = (((np.arccos((-tx)/dT) - np.pi/2) * -1) + kneeHipPaw)
        knee = - np.pi + pawKneeHip
    else:
        hip = (((np.arccos((-tx)/dT) - np.pi/2) * -1) - kneeHipPaw)
        knee = np.pi - pawKneeHip
    
    phi = np.arccos(ty/dT)
    print("phi: " + str(phi))
    return knee, hip, phi

def moveLeg(actuator, position, actType, backLeg):
    global globalHipStatus
    global globalKneeStatus
    xPos = position[0]
    yPos = position[1]
    zPos = position[2]

    hipRotTarg = calcActRotation(xPos,yPos, zPos, backLeg)[1]
    kneeRotTarg = calcActRotation(xPos,yPos,zPos, backLeg)[0]
    rollRotTarg = calcActRotation(xPos,yPos, zPos, backLeg)[2]
    
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

    if actType == 'roll':
        if rollRotTarg < data.ctrl[actuator]:
            data.ctrl[actuator]-= motorSpeed
        else:
            data.ctrl[actuator] += motorSpeed
    

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
        positionTargetZ = 0.1
        moveLeg(FLH, [positionTargetX,positionTargetY, positionTargetZ],'hip', False)
        moveLeg(FLK, [positionTargetX,positionTargetY, positionTargetZ],'knee', False)

        moveLeg(FRH, [positionTargetX,positionTargetY, positionTargetZ],'hip', False)
        moveLeg(FRK, [positionTargetX,positionTargetY,positionTargetZ],'knee', False)

        moveLeg(BLH, [-positionTargetX,positionTargetY,positionTargetZ],'hip', True)
        moveLeg(BLK, [-positionTargetX,positionTargetY,positionTargetZ],'knee', True)

        moveLeg(BRH, [-positionTargetX,positionTargetY,positionTargetZ],'hip', True)
        moveLeg(BRK, [-positionTargetX,positionTargetY,positionTargetZ],'knee', True)
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
walkingPosY = 0.34
walkingLiftPosY = 0.33
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
        positionTargetX = 0.02
        positionTargetY = .34
        positionTargetZ = 0.1
        moveLeg(FLH, [positionTargetX,positionTargetY,positionTargetZ],'hip', False)
        moveLeg(FLK, [positionTargetX,positionTargetY,positionTargetZ],'knee', False)
        moveLeg(FLR, [positionTargetX,positionTargetY,positionTargetZ],'roll', False)

        moveLeg(FRH, [positionTargetX,positionTargetY,positionTargetZ],'hip', False)
        moveLeg(FRK, [positionTargetX,positionTargetY,positionTargetZ],'knee', False)
        moveLeg(FRR, [positionTargetX,positionTargetY,positionTargetZ],'roll', False)

        moveLeg(BLH, [positionTargetX,positionTargetY,positionTargetZ],'hip', True)
        moveLeg(BLK, [positionTargetX,positionTargetY,positionTargetZ],'knee', True)
        moveLeg(BLR, [positionTargetX,positionTargetY,positionTargetZ],'roll', True)


        moveLeg(BRH, [positionTargetX,positionTargetY,positionTargetZ],'hip', True)
        moveLeg(BRK, [positionTargetX,positionTargetY,positionTargetZ],'knee', True)
        moveLeg(BRR, [positionTargetX,positionTargetY,positionTargetZ],'roll', True)

        if self.check_position(FRK, 'knee', positionTargetX, positionTargetY, positionTargetZ, False) \
                and self.check_position(BRK, 'knee', positionTargetX, positionTargetY, positionTargetZ, True):
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
        moveLeg(BLK, [walkingPosX * -d, walkingPosY], 'knee', True)
        moveLeg(BLH, [walkingPosX * -d, walkingPosY], 'hip', True)
        
        if self.check_position(FRK, 'knee', walkingPosX * d, walkingPosY, False) \
                and self.check_position(BLK, 'knee', walkingPosX * -d, walkingPosY, True):
            self.state = 'FR_BL Dropped'


    def FL_BR_Lift(self, d):
        global walkingLiftPosX, walkingLiftPosY, rearOffset
        moveLeg(FLK, [walkingLiftPosX * d, walkingLiftPosY], 'knee', False)
        moveLeg(FLH, [walkingLiftPosX * d, walkingLiftPosY], 'hip', False)
        moveLeg(BRK, [walkingLiftPosX * -d, walkingLiftPosY], 'knee', True)
        moveLeg(BRH, [walkingLiftPosX * -d, walkingLiftPosY], 'hip', True)
        
        if self.check_position(FLK, 'knee', walkingLiftPosX * d, walkingLiftPosY, False) \
                and self.check_position(BRK, 'knee', walkingLiftPosX * -d, walkingLiftPosY, True):
            self.state = 'FL_BR Lifted'

    def FL_BR_Drop(self, d):
        # global walkingPosStatus
        global walkingPosX, walkingPosY, rearOffset
        # if walkingPosStatus == 'FL BR Lifted' or walkingPosStatus == 'Dropping FL BR' or walkingPosStatus == 'none':
            # walkingPosStatus = 'Dropping FL BR'
        moveLeg(FLK, [walkingPosX * d, walkingPosY], 'knee', False)
        moveLeg(FLH, [walkingPosX * d, walkingPosY], 'hip', False)
        moveLeg(BRK, [walkingPosX * -d, walkingPosY], 'knee', True)
        moveLeg(BRH, [walkingPosX * -d, walkingPosY], 'hip', True)
        
        if self.check_position(FLK, 'knee', walkingPosX * d, walkingPosY, False) \
                and self.check_position(BRK, 'knee', walkingPosX * -d, walkingPosY, True):
            # walkingPosStatus = 'FL BR Dropped'
            self.state = 'FL_BR Dropped'


    def check_position(self, actIndex, actType, posX, posY, posZ,backLeg):
        global targetPosPub
        global currentPosPub
        resultIndex = 0
        if actType == 'hip':
            resultIndex = 1
        targetPos = calcActRotation(posX, posY, posZ, backLeg)[resultIndex]
        targetPosPub = targetPos
        currentPos = data.ctrl[actIndex]
        currentPosPub = data.ctrl
        if currentPos >= targetPos - 0.001 and currentPos <= targetPos + 0.001:
            return True
        return False

robot_fsm = RobotStateMachine()

def getCoM():
    total_mass = 0
    com = np.zeros(3)
    for i in range(model.nbody):
        mass = model.body_mass[i]
        pos = data.xipos[i]
        com += mass * pos
        total_mass += mass
    com /= total_mass
    print("Center of Mass:", com)
    return com

# Add the CoM sphere
com_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "com_sphere")


lookPosX = 0
lookPosY = 0

# robotFSM = FiniteStateMachine()
while not glfw.window_should_close(window):
    time_prev = data.time
    direction = 0
    while data.time - time_prev < 1.0/displayRefreshRate:
        counter += 1
        # robotFSM.step()
        # getCoM()
        # data.geom_xpos[model.ngeom - 1] = getCoM
        total_mass = 0
        com = np.zeros(3)
        for i in range(model.nbody):
            mass = model.body_mass[i]
            pos = data.xipos[i]
            com += mass * pos
            total_mass += mass
        com /= total_mass
        # print("CoM: " + str(com))
        # print("CoM Sphere ID:", com_geom_id)
        data.xpos[com_body_id] = com
        # print(f"Updated CoM Sphere Position: {data.geom_xpos[com_geom_id]}")
        
        # robot_fsm.step()
        robot_fsm.neutralPos()
        # neutralPos()
        # print(f"CoM Sphere xpos: {data.xpos[com_body_id]}")
        # robot_fsm.neutralPos()
        
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
    cam.azimuth = 0  # Keep or modify this for different angles
    cam.elevation = -0 # Adjust the elevation if necessary
    cam.orthographic = 1
    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()