from Joint_Controller.HardwareInterface import CanBus, Motor

import time

bus = CanBus()
bus.start()

# motor_ids = [1, 4, 7, 10] #hips
motor_ids = [7, 8, 9] #knees
# motor_ids = [3, 6, 9, 12] #roll

for i in motor_ids:
    motor = Motor(bus, i)
    motor.reset_encoder()
    
bus.close()