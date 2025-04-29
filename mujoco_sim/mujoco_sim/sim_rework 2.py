import mujoco as mj
from mujoco.glfw import glfw
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray



print_camera_config = 1 #set to 1 to print camera config
                        #this is useful for initializing view of the model

modelPath = '/home/jc/BruinBear/ros2_ws/quadruped-new/quadruped_o.xml'
displayRefreshRate = 60
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
    ('Actuator Positions: "%s"' % msg.data)

#Rangefinder publisher
class RangefinderPub(Node):
  def __init__(self):
    super().__init__('rangefinder_pub')
    self.publisher_ = self.create_publisher(Float32MultiArray, 'rangefinder_data', 10)
    timer_period = 0.5
    # self.timer = self.create_timer(timer_period, self.pub_rangefinder_data)

  def pub_rangefinder_data(self):
    msg = Float32MultiArray()
    if(getRange('range1') < 2000):
        msg.data.append(getRange('range2'))
    else:
        msg.data.append(-1)
    if(getRange('range2') < 2000):
        msg.data.append(getRange('range1'))
    else:
        msg.data.append(-1)
    if(getRange('range3') < 2000):
        msg.data.append(getRange('range3'))
    else:
        msg.data.append(-1)
    self.publisher_.publish(msg)
    self.get_logger().info("publishing: " + str(msg)) 
    # print("\033c") # disable if this causes problems, just clears the terminal  
    # ('Rangefinder Data: "%s"' % msg.data)

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
        elif key == glfw.KEY_N:
            cubeControl3 = 'cube_rot_ccw'
        elif key == glfw.KEY_:
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
        elif key == glfw.KEY_N:
            cubeControl3 = 'none'
        elif key == glfw.KEY_P:
            cubeControl3 = 'none'

# MuJoCo data structures
model = mj.MjModel.from_xml_path(modelPath)  # MuJoCo model
data = mj.MjData(model)                     # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options


#set the controller
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

cube1 = mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT, 'box-joint1')
cube_qpos_addr1 = model.jnt_qposadr[cube1]  # Get qpos index for the cube joint

cube2 = mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT, 'box-joint2')
cube_qpos_addr2 = model.jnt_qposadr[cube2]  # Get qpos index for the cube joint

cube3 = mj.mj_name2id(model,mj.mjtObj.mjOBJ_JOINT, 'box-joint3')
cube_qpos_addr3 = model.jnt_qposadr[cube3]  # Get qpos index for the cube joint

com_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_BODY, 'com-sphere')


# com_sphere_addr = model.jnt_gposadr


rclpy.init()
actuatorPositionNode = ActuatorPositionPub()
randgefinderNode = RangefinderPub()
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

    hip = np.arccos(-tx/dT) - kneeHipPaw
    knee = np.pi - pawKneeHip

    if backleg:
        hip = (((np.arccos((-tx)/dT) - np.pi/2) * -1) + kneeHipPaw)
        knee = - np.pi + pawKneeHip
    else:
        hip = (((np.arccos((-tx)/dT) - np.pi/2) * -1) - kneeHipPaw)
        knee = np.pi - pawKneeHip
    
    phi = np.arccos(ty/dT)
    if(tz < 0):
        phi *= -1

    if knee > np.deg2rad(60):
        print('out of range')
    return knee, hip, phi

def moveLeg(actuator, position, actType, backLeg):
    xPos = position[0]
    yPos = position[1]
    zPos = position[2]

    hipRotTarg = calcActRotation(xPos,yPos, zPos, backLeg)[1]
    kneeRotTarg = calcActRotation(xPos,yPos,zPos, backLeg)[0]
    rollRotTarg = calcActRotation(xPos,yPos, zPos, backLeg)[2]
    
    if actType == 'hip':
        if hipRotTarg < data.ctrl[actuator]:
            data.ctrl[actuator] -= motorSpeed
        else:
            data.ctrl[actuator] += motorSpeed

    if actType == 'knee':
        if kneeRotTarg < data.ctrl[actuator]:
            data.ctrl[actuator] -= motorSpeed
        else:
            data.ctrl[actuator] += motorSpeed

    if actType == 'roll':
        if rollRotTarg < data.ctrl[actuator]:
            data.ctrl[actuator]-= motorSpeed
        else:
            data.ctrl[actuator] += motorSpeed
    


walkCounter = 0
walkingPosX = 0.03
walkingLiftPosX = walkingPosX/2
walkingPosY = 0.35
walkingLiftPosY = 0.335
walkingPosZ = 0.0
turnOffset = 0.1
class RobotStateMachine:
    walkCounter = 0
    def __init__(self):
        self.state = 'get neutral'

    def step(self, direction):
        # print(self.state)
        if (self.state == 'get neutral'):
            self.neutralPos()
        elif self.state == 'INIT':
            self.FL_BR_Drop(-direction)
            self.FR_BL_Lift(direction)
        elif self.state == 'FR_BL Lifted':
            self.FR_BL_Drop(direction)
        elif self.state == 'FR_BL Dropped':
            self.FR_BL_Drop(-direction)
            self.FL_BR_Lift(direction)
        elif self.state == 'FL_BR Lifted':
            self.FL_BR_Drop(direction)
        elif self.state == 'FL_BR Dropped':
            # print("Sequence complete: Resetting to INIT")
            self.walkCounter += 1
            self.state = 'INIT'  # Reset or set up for next step

    def turn(self, direction):
        oppositeDirection = 'right'
        if direction == 'right':
            oppositeDirection = 'left'
        else:
            oppositeDirection = 'right'

        if(self.state == 'get neutral'):
            self.neutralPos()
        elif self.state == 'INIT':
            self.liftDiagonals(f'{direction}', 'extended')
        elif self.state == f'{direction}_diagonals_lifted':
            self.extendDiagonal(f'{direction}',1)
        elif self.state == f'{direction}_extended1':
            self.extendDiagonal(f'{oppositeDirection}',1)
        elif self.state == f'{oppositeDirection}_extended1':
            self.retractDiag(f'{direction}',1)
        elif self.state == f'{direction}_retracted1':
            self.retractDiag(f'{oppositeDirection}',2)
        elif self.state == f'{oppositeDirection}_retracted2':
            self.state = "INIT"

    def neutralPos(self):
        positionTargetX = 0.02
        positionTargetY = .34
        positionTargetZ = 0.0
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
        global walkingLiftPosX, walkingLiftPosY, walkingPosZ
        moveLeg(FRK, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'knee', False)
        moveLeg(FRH, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'hip', False)
        moveLeg(FRR, [walkingLiftPosX * d,walkingLiftPosY,walkingPosZ],'roll', False)

        moveLeg(BLK, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'knee', True)
        moveLeg(BLH, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'hip', True)
        moveLeg(BLR, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'roll', True)
      

        
        if self.check_position(FRK, 'knee', walkingLiftPosX * d, walkingLiftPosY, walkingPosZ, False) \
                and self.check_position(BLK, 'knee', walkingLiftPosX * d, walkingLiftPosY,walkingPosZ, True):
            self.state = 'FR_BL Lifted'

    def FR_BL_Drop(self, d):
        global walkingPosX, walkingPosY, walkingPosZ
        moveLeg(FRK, [walkingPosX * d, walkingPosY, walkingPosZ], 'knee', False)
        moveLeg(FRH, [walkingPosX * d, walkingPosY, walkingPosZ], 'hip', False)
        moveLeg(FRR, [walkingPosX * d, walkingPosY, walkingPosZ], 'roll', False)

        moveLeg(BLK, [walkingPosX * d, walkingPosY, walkingPosZ], 'knee', True)
        moveLeg(BLH, [walkingPosX * d, walkingPosY, walkingPosZ], 'hip', True)
        moveLeg(BLR, [walkingPosX * d, walkingPosY, walkingPosZ], 'roll', True)
        
        if self.check_position(FRK, 'knee', walkingPosX * d, walkingPosY, walkingPosZ, False) \
                and self.check_position(BLK, 'knee', walkingPosX * d, walkingPosY,walkingPosZ, True):
            self.state = 'FR_BL Dropped'


    def FL_BR_Lift(self, d):
        global walkingLiftPosX, walkingLiftPosY, walkingPosZ
        moveLeg(FLK, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'knee', False)
        moveLeg(FLH, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'hip', False)
        moveLeg(FLR, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'roll', False)

        moveLeg(BRK, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'knee', True)
        moveLeg(BRH, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'hip', True)
        moveLeg(BRR, [walkingLiftPosX * d, walkingLiftPosY, walkingPosZ], 'roll', True)
        
        if self.check_position(FLK, 'knee', walkingLiftPosX * d, walkingLiftPosY, walkingPosZ, False) \
                and self.check_position(BRK, 'knee', walkingLiftPosX * d, walkingLiftPosY, walkingPosZ, True):
            self.state = 'FL_BR Lifted'

    def FL_BR_Drop(self, d):
        global walkingPosX, walkingPosY, walkingPosZ
        moveLeg(FLK, [walkingPosX * d, walkingPosY, walkingPosZ], 'knee', False)
        moveLeg(FLH, [walkingPosX * d, walkingPosY, walkingPosZ], 'hip', False)
        moveLeg(FLR, [walkingPosX * d, walkingPosY, walkingPosZ], 'roll', False)

        moveLeg(BRK, [walkingPosX * d, walkingPosY, walkingPosZ], 'knee', True)
        moveLeg(BRH, [walkingPosX * d, walkingPosY, walkingPosZ], 'hip', True)
        moveLeg(BRR, [walkingPosX * d, walkingPosY, walkingPosZ], 'roll', True)
        
        if self.check_position(FLK, 'knee', walkingPosX * d, walkingPosY, walkingPosZ, False) \
                and self.check_position(BRK, 'knee', walkingPosX * d, walkingPosY, walkingPosZ, True):
            self.state = 'FL_BR Dropped'

    def liftDiagonals(self, side, previousZ):
        global walkingLiftPosY, walkingPosY, turnOffset
        if previousZ == 'extended':
            zVal = turnOffset
        else:
            zVal = 0

        if side =='right':
            moveLeg(FRK, [0, walkingLiftPosY, zVal], 'knee', False)
            moveLeg(FRH, [0, walkingLiftPosY, zVal], 'hip', False)
            moveLeg(FRR, [0, walkingLiftPosY, zVal], 'roll', False)

            moveLeg(BLK, [0, walkingLiftPosY, zVal], 'knee', True)
            moveLeg(BLH, [0, walkingLiftPosY, zVal], 'hip', True)
            moveLeg(BLR, [0, walkingLiftPosY, zVal], 'roll', True)
        else:
            moveLeg(FLK, [0, walkingLiftPosY, zVal], 'knee', False)
            moveLeg(FLH, [0, walkingLiftPosY, zVal], 'hip', False)
            moveLeg(FLR, [0, walkingLiftPosY, zVal], 'roll', False)

            moveLeg(BRK, [0, walkingLiftPosY, zVal], 'knee', True)
            moveLeg(BRH, [0, walkingLiftPosY, zVal], 'hip', True)
            moveLeg(BRR, [0, walkingLiftPosY, zVal], 'roll', True)
       
        if side == 'right':
            if self.check_position(FRK, 'knee', 0, walkingLiftPosY, zVal, False) \
                    and self.check_position(BLK, 'knee', 0, walkingLiftPosY, zVal, True):
                self.state = 'right_diagonals_lifted'
        else:
            if self.check_position(FLK, 'knee', 0, walkingLiftPosY, zVal, False) \
                    and self.check_position(BRK, 'knee', 0, walkingLiftPosY, zVal, True):
                self.state = 'left_diagonals_lifted'


    def dropDiagonals(self, side, previousZ):
        global walkingLiftPosY, walkingPosY, turnOffset
        if previousZ == 'extended':
            zVal = turnOffset
        else:
            zVal = 0

        if side =='left':
            moveLeg(FLK, [0, walkingPosY, zVal], 'knee', False)
            moveLeg(FLH, [0, walkingPosY, zVal], 'hip', False)
            moveLeg(FLR, [0, walkingPosY, zVal], 'roll', False)

            moveLeg(BRK, [0, walkingPosY, zVal], 'knee', True)
            moveLeg(BRH, [0, walkingPosY, zVal], 'hip', True)
            moveLeg(BRR, [0, walkingPosY, zVal], 'roll', True)
        else:
            moveLeg(FRK, [0, walkingPosY, zVal], 'knee', False)
            moveLeg(FRH, [0, walkingPosY, zVal], 'hip', False)
            moveLeg(FRR, [0, walkingPosY, zVal], 'roll', False)

            moveLeg(BLK, [0, walkingPosY, zVal], 'knee', True)
            moveLeg(BLH, [0, walkingPosY, zVal], 'hip', True)
            moveLeg(BLR, [0, walkingPosY, zVal], 'roll', True)
    
        if side == 'right':
            if self.check_position(FRK, 'knee', 0, walkingPosY, zVal, False) \
                    and self.check_position(BLK, 'knee', 0, walkingPosY, zVal, True):
                self.state = 'right_dropped'
        else:
            if self.check_position(FLK, 'knee', 0, walkingPosY, zVal, False) \
                    and self.check_position(BRK, 'knee', 0, walkingPosY, zVal, True):
                self.state = 'left_dropped'


        
    def extendDiagonal(self, side, mode):
        global turnOffset
        if side =='left':
            moveLeg(FLK, [0, walkingPosY, turnOffset], 'knee', False)
            moveLeg(FLH, [0, walkingPosY, turnOffset], 'hip', False)
            moveLeg(FLR, [0, walkingPosY, turnOffset], 'roll', False)

            moveLeg(BRK, [0, walkingPosY, turnOffset], 'knee', True)
            moveLeg(BRH, [0, walkingPosY, turnOffset], 'hip', True)
            moveLeg(BRR, [0, walkingPosY, turnOffset], 'roll', True)
           
        else:
            moveLeg(FRK, [0, walkingPosY, turnOffset], 'knee', False)
            moveLeg(FRH, [0, walkingPosY, turnOffset], 'hip', False)
            moveLeg(FRR, [0, walkingPosY, turnOffset], 'roll', False)

            moveLeg(BLK, [0, walkingPosY, turnOffset], 'knee', True)
            moveLeg(BLH, [0, walkingPosY, turnOffset], 'hip', True)
            moveLeg(BLR, [0, walkingPosY, turnOffset], 'roll', True)

       
        if side == 'right':
            if self.check_position(FRK, 'knee', 0, walkingPosY, turnOffset, False) \
                    and self.check_position(BLK, 'knee', 0, walkingPosY, turnOffset, True):
                self.state = 'right_extended' + str(mode)
        else:
            if self.check_position(FLK, 'knee', 0, walkingPosY, turnOffset, False) \
                    and self.check_position(BRK, 'knee', 0, walkingPosY, turnOffset, True):
                self.state = 'left_extended' + str(mode)

    def retractDiag(self,side, mode):
        if side =='left':
            moveLeg(FLK, [0, walkingPosY, 0], 'knee', False)
            moveLeg(FLH, [0, walkingPosY, 0], 'hip', False)
            moveLeg(FLR, [0, walkingPosY, 0], 'roll', False)

            moveLeg(BRK, [0, walkingPosY, 0], 'knee', True)
            moveLeg(BRH, [0, walkingPosY, 0], 'hip', True)
            moveLeg(BRR, [0, walkingPosY, 0], 'roll', True)
           
        else:
            moveLeg(FRK, [0, walkingPosY, 0], 'knee', False)
            moveLeg(FRH, [0, walkingPosY, 0], 'hip', False)
            moveLeg(FRR, [0, walkingPosY, 0], 'roll', False)

            moveLeg(BLK, [0, walkingPosY, 0], 'knee', True)
            moveLeg(BLH, [0, walkingPosY, 0], 'hip', True)
            moveLeg(BLR, [0, walkingPosY, 0], 'roll', True)

        if side == 'right':
            if self.check_position(FRK, 'knee', 0, walkingPosY, 0, False) \
                    and self.check_position(BLK, 'knee', 0, walkingPosY, 0, True):
                self.state = 'right_retracted' + str(mode)
        else:
            if self.check_position(FLK, 'knee', 0, walkingPosY, 0, False) \
                    and self.check_position(BRK, 'knee', 0, walkingPosY, 0, True):
                self.state = 'left_retracted' + str(mode)

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
    for i in range(model.nbody -3):
        mass = model.body_mass[i]
        pos = data.xipos[i]
        com += mass * pos
        total_mass += mass
    com /= total_mass
    # print("Center of Mass:", com)
    return com

# Add the CoM sphere
# com_body_id = mj.mj_name2id(model, mj.mjtObj.mjOBJ_GEOM, "com_sphere")

#Add code for obtaining data from rangefinder
def getRange(rangefinder_name):
    # mujoco.mj_resetData(model, data)
    # data.ctrl = 20
    return data.sensor(rangefinder_name).data.copy()

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(int(1200*0.6), int(900*0.6), "Quadruped", None, None)
glfw.make_context_current(window)
# glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard_callback)

cam.distance = 2
cam.azimuth = 45
cam.elevation = -35
cam.orthographic = 1
robot_fsm.__init__
while not glfw.window_should_close(window):
    time_prev = data.time
    direction = 0

    while data.time - time_prev < 1.0 / displayRefreshRate:
        randgefinderNode.pub_rangefinder_data()
        # Keyboard Control
        if keyBoardControl == 'fwd':
            robot_fsm.step(1)
        elif keyBoardControl == 'lft':
            robot_fsm.turn("left")
        elif keyBoardControl == 'rgt':
            robot_fsm.turn('right')
        elif keyBoardControl == 'bck':
            robot_fsm.step(-1)
        elif keyBoardControl == 'nut':
            robot_fsm.neutralPos()

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
        # # Step the MuJoCo simulation
        # robot_fsm.turn('left')

        data.mocap_pos[0] = getCoM()
        mj.mj_step(model, data)
        glfw.poll_events()

        # Add rendering or debugging code here if needed
        if counter % 100 == 0:
            pass
            # Add any periodic debugging or state updates here

    # Handle rendering
    viewport_width, viewport_height = glfw.get_framebuffer_size(window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    # Camera updates (tracking robot position)
    robot_x = data.xpos[1][0]  # X-coordinate of the robot
    robot_y = data.xpos[1][1]  # Y-coordinate of the robot
    cam.lookat = [robot_x, robot_y, 0.2]



    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam, mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # Swap buffers
    glfw.swap_buffers(window)

glfw.terminate()