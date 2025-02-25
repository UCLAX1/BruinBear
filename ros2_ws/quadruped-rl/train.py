import gymnasium as gym
from stable_baselines3 import PPO
from quadruped_env import QuadrupedEnv

env = QuadrupedEnv(xml_path="/home/sara/Documents/BruinBear/ros2_ws/quadruped-new/quadruped.xml", render_mode="human")


# Load PPO agent
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_quadruped/")

# Train for 1 million steps
model.learn(total_timesteps=1_000_000)

# Save the trained model
model.save("ppo_quadruped")
