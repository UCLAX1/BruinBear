import time
from Joint_Controller.hip import Hip
from Joint_Controller.knee import Knee
from Joint_Controller.roll import Roll
from Joint_Controller.HardwareInterface import CanBus, Motor

class LegController:
    def __init__(self):
        self.joint_positions = None
        
        bus = CanBus()
        bus.start()
        
        self.FLHip = Hip(1, bus, inverted=False)
        self.FLKnee = Knee(2, bus)
        self.FLRoll = Roll(3, bus)

        self.FRHip = Hip(4, bus, inverted=True)
        self.FRKnee = Knee(5, bus)
        self.FRRoll = Roll(6, bus)

        self.BRHip = Hip(7, bus, inverted=False)
        self.BRKnee = Knee(8, bus)
        self.BRRoll = Roll(9, bus)

        self.BLHip = Hip(10, bus, inverted=True)
        self.BLKnee = Knee(11, bus)
        self.BLRoll = Roll(12, bus)
        
            
    def update(self, joint_positions):     
        self.joint_positions = joint_positions
           
        if joint_positions is None:
            return
        
        self.FLHip.set_target_rad(joint_positions[0])
        self.FLKnee.set_target_rad(joint_positions[4])
        self.FLRoll.set_target_rad(joint_positions[8])

        self.FRHip.set_target_rad(joint_positions[1])
        self.FRKnee.set_target_rad(joint_positions[5])
        self.FRRoll.set_target_rad(joint_positions[9])


        self.BRHip.set_target_rad(joint_positions[2])
        self.BRKnee.set_target_rad(joint_positions[6])
        self.BRRoll.set_target_rad(joint_positions[10])

        self.BLHip.set_target_rad(joint_positions[3])
        self.BLKnee.set_target_rad(joint_positions[7])
        self.BLRoll.set_target_rad(joint_positions[11])

        
        self.FLKnee.update_motor_power()
        self.FLHip.update_motor_power()
        self.FLRoll.update_motor_power()

        self.FRKnee.update_motor_power()
        self.FRHip.update_motor_power()
        self.FRRoll.update_motor_power()

        self.BRKnee.update_motor_power()
        self.BRHip.update_motor_power()
        self.BRRoll.update_motor_power()

        self.BLKnee.update_motor_power()
        self.BLHip.update_motor_power()
        self.BLRoll.update_motor_power()



        # Log cycle time every 1000 iterations
        cycle += 1
        if cycle % 1000 == 0:
            current_time = time.time()
            cycle_time = (current_time - last_time) / 1000  # Average time per cycle
            print(f"Average cycle time: {cycle_time:.6f} seconds")
            last_time = current_time
