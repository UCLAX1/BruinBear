from HardwareInterface import CanBus, Motor
    
import time

bus = CanBus()
bus.start()
motor = Motor(bus, 1)

motor.set_power(-0.5)

for i in range(20):
    motor.send_hearbeat()
    print(f"Motor position: {motor.get_pos()}")
    time.sleep(0.1)
    
bus.close()
