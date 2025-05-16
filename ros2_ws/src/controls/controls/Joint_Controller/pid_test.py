try: 
    from controls.Joint_Controller.HardwareInterface import CanBus, Motor
    from controls.Joint_Controller.hip import Hip
    from controls.Joint_Controller.knee import Knee
except ImportError:
    from HardwareInterface import CanBus, Motor
    from hip import Hip
    from knee import Knee
import time

bus = CanBus()
bus.start()

testMotorK = Knee(1, bus)
testMotorH = Hip(2, bus)

time.sleep(2)

testMotorK.set_target_ticks(0)
testMotorH.set_target_ticks(0)
while True:
    # testMotorK.update_motor_power()
    # testMotorH.update_motor_power()
    print(f"Motor position Knee: {testMotorK.get_current_ticks()}")
    print(f"Motor position Hip: {testMotorH.get_current_ticks()}")
    time.sleep(0.2)
    
bus.close()
