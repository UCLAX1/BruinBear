from HardwareInterface import CanBus, Motor
from hip import Hip
import time

bus = CanBus()
bus.start()

testHip = Hip(1, bus)

time.sleep(2)

testHip.set_target_position(100000)
for i in range(50):
    if i == 25:
        testHip.set_target_position(0)
    testHip.update_motor_power()
    print(f"Motor position: {testHip.get_current_position()}")
    time.sleep(0.1)
    
bus.close()
