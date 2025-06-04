try: 
    from HardwareInterface import CanBus, Motor
except ImportError:
    from controls.Joint_Controller.HardwareInterface import CanBus, Motor
    
import time

bus = CanBus()
bus.start()

motor_ids = [1, 2, 3]

for i in motor_ids:
    motor = Motor(bus, i)
    motor.reset_encoder()
    
bus.close()