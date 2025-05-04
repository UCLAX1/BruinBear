def controlObstacle(cubeControl,data, cube_address):
    match cubeControl:
        case 'cube_fwd':
            data.qpos[cube_address] += 0.001
        case 'cube_bck':
            data.qpos[cube_address] -= 0.001
        case 'cube_right':
            data.qpos[cube_address + 1] += 0.001
        case 'cube_left':
            data.qpos[cube_address + 1] -= 0.001
        case 'cube_rot_cw':
            data.qpos[cube_address + 6] += 0.001
        case 'cube_rot_cw':
            data.qpos[cube_address + 6] -= 0.001

def controlRobot(keyboard_state, robot_fsm):
    match keyboard_state:
        case 'fwd':
            robot_fsm.step(1)
        case 'bck':
            robot_fsm.step(-1)
        case 'lft':
            robot_fsm.turn('left')
        case 'rgt':
            robot_fsm.turn('right')
        case 'nut':
            robot_fsm.neutralPos()

def controlCamera(camera_state, cam):
    match camera_state:
        case 'down':
            cam.elevation -= 0.1
        case 'up':
            cam.elevation += 0.1
        case 'right':
            cam.azimuth -= 0.1
        case 'left':
            cam.azimuth += 0.1
        case 'home':
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