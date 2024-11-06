import time
import mujoco
import mujoco.viewer

# Load the model
model = mujoco.MjModel.from_xml_path('/Users/sara/Documents/My-Documents/X1-Robotics/BruinBear/Quadruped-XML/quadruped.xml')
data = mujoco.MjData(model)

# Get actuator ID
FRH = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'FR-H-servo')
if FRH == -1:
    raise ValueError("Actuator 'FR-H-servo' not found in the model XML.")

# Target position for the actuator
target_position = 0.5
steps_per_iteration = 10  # Number of steps to run per loop iteration

# Use the viewer's managed context
with mujoco.viewer.launch(model, data) as viewer:
    print("Running simulation with viewer...")

    for step in range(1000):  # Run for 1000 steps as a test
        # Apply control command to reach target position
        data.ctrl[FRH] = target_position

        # Run multiple steps per iteration for faster simulation
        for _ in range(steps_per_iteration):
            mujoco.mj_step(model, data)

        # Print debug information
        joint_id = model.jnt_qposadr[FRH]  # Joint position index for the actuator
        print(f"Step {step}: qpos={data.qpos[joint_id]} (target: {target_position})")

        # Sync the viewer
        viewer.sync()

        # Small delay for readability of printed output
        time.sleep(0.01)