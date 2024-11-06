import time
import mujoco

# Load the model
model = mujoco.MjModel.from_xml_path('/Users/sara/Documents/My-Documents/X1-Robotics/BruinBear/Quadruped-XML/quadruped.xml')
data = mujoco.MjData(model)

# Get actuator ID
FRH = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, 'FR-H-servo')
if FRH == -1:
    raise ValueError("Actuator 'FR-H-servo' not found in the model XML.")

# Target position for the actuator
target_position = 0.5
print("Moving actuator to position:", target_position)

# Run simulation loop
for step in range(100000):  # Run for 1000 steps as a test
    # Set the target position
    data.ctrl[FRH] = target_position

    # Advance simulation step
    mujoco.mj_step(model, data)

    # Print debug information
    joint_id = model.jnt_qposadr[FRH]  # Get associated joint position ID
    print(f"Step {step}: qpos={data.qpos[joint_id]} (target: {target_position})")

    # Check if the joint is near the target position
    if abs(data.qpos[joint_id] - target_position) < 0.01:
        print("Reached target position!")
        break

    # time.sleep(0.01)