from robot import Robot
import time

robot = Robot(gait="sit", useMotors=True, useSim=False)
startTime = time.time()

while True:
    if time.time() - startTime >= 1:
        robot.gait = "f"
    robot.update()
