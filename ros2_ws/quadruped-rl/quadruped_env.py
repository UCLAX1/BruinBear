import numpy as np
import gymnasium as gym
from gymnasium import spaces
import mujoco
from mujoco.glfw import glfw
# import gym
# from gym import spaces
cam = mujoco.MjvCamera()    
opt = mujoco.MjvOption()  
glfw.init()
window = glfw.create_window(int(1200*0.6), int(900*0.6), "Quadruped", None, None)
glfw.make_context_current(window)


class QuadrupedEnv(gym.Env):
    def __init__(self, xml_path, render_mode="human"):
        global window
        self.render_mode = render_mode
   
                    # Abstract camera


        # Load MuJoCo model from the provided XML path
        
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        # self.window = glfw.create_window(int(1200*0.6), int(900*0.6), "Quadruped", None, None)

        self.scene = mujoco.MjvScene(self.model, maxgeom=10000)
        self.viewport_width, self.viewport_height = glfw.get_framebuffer_size(window)
        self.viewport = mujoco.MjrRect(0, 0, self.viewport_width, self.viewport_height)
        self.context = mujoco.MjrContext(self.model, mujoco.mjtFontScale.mjFONTSCALE_150.value)




        # Define action and observation space
        self.action_space = spaces.Box(low=-1, high=1, shape=(19,), dtype=float)
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=(19,), dtype=np.float32)

        # Initialize MuJoCo viewer if rendering is enabled
        if self.render_mode == "human":
            # glfw.init()
            # window = glfw.create_window(int(1200*0.6), int(900*0.6), "Quadruped", None, None)
            # glfw.make_context_current(window)
            mujoco.mjv_defaultCamera(cam)
            mujoco.mjv_defaultOption(opt)
 

    def step(self, action):
        mujoco.mj_step(self.model, self.data)
        
        if self.render_mode == "human":
           self.render()


        obs = self._get_obs()
        reward = self._compute_reward()
        done = self._check_termination()
        truncated = False
        info = {}
        return obs, reward, done, truncated, info

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        obs = self._get_obs()
        print(f"Reset Observation Shape: {obs.shape}")  # Debugging line
        return obs, {}



    def _get_obs(self):
        return self.data.qpos.copy()

    def _compute_reward(self):
        return 0  # Replace with actual reward function

    def _check_termination(self):
        return False  # Replace with termination condition

    def render(self):
        global opt,cam, window
        # while not glfw.window_should_close(window):
        mujoco.mjv_updateScene(self.model, self.data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL.value, self.scene)
        mujoco.mjr_render(self.viewport, self.scene, self.context)  # Ensure rendering updates
        glfw.swap_buffers(window)
        glfw.poll_events()

    def close(self):
        if self.render_mode == "human":
            self.viewer.close()