import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R

class Sim:
    def __init__(self, startPos, displayRefreshRate = 30):
        
        modelPath = 'ros2_ws/quadruped-new/quadruped.xml'
        self.displayRefreshRate = displayRefreshRate
        self.joints = startPos

        # MuJoCo self.data structures
        self.model = mj.MjModel.from_xml_path(modelPath)  # MuJoCo self.model
        self.data = mj.MjData(self.model)                 # MuJoCo self.data
        self.cam = mj.MjvCamera()                        # Abstract camera
        self.opt = mj.MjvOption()                        # visualization options

        # Init GLFW, create window, make OpenGL self.context current, request v-sync
        glfw.init()
        self.window = glfw.create_window(640, 480, "Demo", None, None)
        glfw.make_context_current(self.window)
        glfw.swap_interval(1)

        # initialize visualization self.data structures
        mj.mjv_defaultCamera(self.cam)
        mj.mjv_defaultOption(self.opt)
        self.scene = mj.MjvScene(self.model, maxgeom=10000)
        self.context = mj.MjrContext(self.model, mj.mjtFontScale.mjFONTSCALE_150.value)

        #set the controller
        def controller(model, data):
            #put the controller here. This function is called inside the simulation.
            pass
        
        mj.set_mjcb_control(controller)
        self.FRH = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'FR-H-servo')
        self.FLH = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'FL-H-servo')
        self.BRH = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'BR-H-servo')
        self.BLH = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'BL-H-servo')

        self.FRK = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'FR-K-servo')
        self.FLK = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'FL-K-servo')
        self.BRK = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'BR-K-servo')
        self.BLK = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'BL-K-servo')

        self.FRR = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'FR-R-servo')
        self.FLR = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'FL-R-servo')
        self.BRR = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'BR-R-servo')
        self.BLR = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_ACTUATOR, 'BL-R-servo')
        
        self.cam.distance = 2  # Adjust this distance as needed
        self.cam.azimuth = 45   # Keep or modify this for different angles
        self.cam.elevation = -35 # Adjust the elevation if necessary
    
    # Update positions and self.scene and render
    def update(self, joints):
        if self.is_running():
            time_prev = self.data.time

            while self.data.time - time_prev < 1.0/self.displayRefreshRate:
                self.data.ctrl[self.FLH] = joints[0]
                self.data.ctrl[self.FRH] = joints[1]
                self.data.ctrl[self.BRH] = joints[2]
                self.data.ctrl[self.BLH] = joints[3]
                self.data.ctrl[self.FLK] = joints[4]
                self.data.ctrl[self.FRK] = joints[5]
                self.data.ctrl[self.BRK] = joints[6]
                self.data.ctrl[self.BLK] = joints[7]
                self.data.ctrl[self.FLR] = joints[8]
                self.data.ctrl[self.FRR] = joints[9]
                self.data.ctrl[self.BRR] = joints[10]
                self.data.ctrl[self.BLR] = joints[11]
                
                mj.mj_step(self.model, self.data)

            viewport_width, viewport_height = glfw.get_framebuffer_size(
                self.window)
            viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

            smoothing_factor = 0.1
            robot_x = self.data.xpos[1][0]  # X-coordinate of the robot
            robot_y = self.data.xpos[1][1]  # Y-coordinate of the robot 
            self.cam.lookat = [robot_x, robot_y, 0.2]

            mj.mjv_updateScene(self.model, self.data, self.opt, None, self.cam,
                            mj.mjtCatBit.mjCAT_ALL.value, self.scene)
            mj.mjr_render(viewport, self.scene, self.context)

            # swap OpenGL buffers (blocking call due to v-sync)
            glfw.swap_buffers(self.window)

            # process pending GUI events, call GLFW callbacks
            glfw.poll_events()
        else:
            self.close()
            
        return self.is_running()
            
    def is_running(self):
        return not glfw.window_should_close(self.window)

    def close(self):
        glfw.destroy_window(self.window)
        glfw.terminate()
