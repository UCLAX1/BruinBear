from robot import Robot
import time

robot = Robot(gait="sit")
startTime = time.time()

while True:
    if time.time() - startTime >= 1:
        robot.gait = "f"
    robot.update()
    print(robot.joint_positions)
