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

testMotor = Knee(1, bus)

time.sleep(2)

testMotor.set_target_position(0.5)
# testMotor.apply_motor_power(0)
while True:
    testMotor.update_motor_power()
    print(f"Motor position: {testMotor.get_current_position()}")
    time.sleep(0.1)
    
bus.close()
