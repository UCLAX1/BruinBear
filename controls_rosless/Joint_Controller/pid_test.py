
from HardwareInterface import CanBus, Motor
from hip import Hip
from knee import Knee
from roll import Roll
import time

bus = CanBus()
bus.start()

# testMotorK = Knee(1, bus)
# testMotorH = Hip(2, bus)
testMotorJ = Roll(10, bus)

time.sleep(2)

# testMotorK.set_target_ticks(-5)
# testMotorH.set_target_ticks(4)
# testMotorJ.set_target_ticks(-0.5)
while True:
    # testMotorK.update_motor_power()
    # testMotorH.update_motor_power()
    # testMotorJ.update_motor_power()
    # print(f"Motor position Knee: {testMotorK.get_current_ticks()}")
    # print(f"Motor position Hip: {testMotorH.get_current_ticks()}")
    print(f"Motor position Roll: {testMotorJ.get_current_ticks()}")
    time.sleep(0.05)
    
bus.close()
