from Joint_Controller.HardwareInterface import CanBus, Motor

import time

bus = CanBus()
bus.start()

motor_ids = [4, 5, 6]

for i in motor_ids:
    motor = Motor(bus, i)
    motor.reset_encoder()
    
bus.close()