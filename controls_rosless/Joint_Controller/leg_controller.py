import time
from Joint_Controller.hip import Hip
from Joint_Controller.knee import Knee
from Joint_Controller.roll import Roll
from Joint_Controller.HardwareInterface import CanBus, Motor

class LegController:
    def __init__(self, useFL =True, useFR=True, useBR=True, useBL=True):
        self.joint_positions = None
        self.useFL = useFL
        self.useFR = useFR
        self.useBR = useBR
        self.useBL = useBL
        
        bus = CanBus()
        bus.start()
        
        if self.useFL:
            self.FLHip = Hip(1, bus, inverted=False)
            self.FLKnee = Knee(2, bus)
            self.FLRoll = Roll(3, bus)
        if self.useFR:
            self.FRHip = Hip(4, bus, inverted=True)
            self.FRKnee = Knee(5, bus)
            self.FRRoll = Roll(6, bus)
        if self.useBR:
            self.BRHip = Hip(7, bus, inverted=False)
            self.BRKnee = Knee(8, bus)
            self.BRRoll = Roll(9, bus)
        if self.useBL:
            self.BLHip = Hip(10, bus, inverted=True)
            self.BLKnee = Knee(11, bus)
            self.BLRoll = Roll(12, bus)
        
            
    def update(self, joint_positions):     
        self.joint_positions = joint_positions
           
        if joint_positions is None:
            return
        
        if self.useFL:
            self.FLHip.set_target_rad(joint_positions[0])
            self.FLKnee.set_target_rad(joint_positions[4])
            self.FLRoll.set_target_rad(joint_positions[8])
        
            self.FLKnee.update_motor_power()
            self.FLHip.update_motor_power()
            self.FLRoll.update_motor_power()
        
        if self.useFR:
            self.FRHip.set_target_rad(joint_positions[1])
            self.FRKnee.set_target_rad(joint_positions[5])
            self.FRRoll.set_target_rad(joint_positions[9])

            self.FRKnee.update_motor_power()
            self.FRHip.update_motor_power()
            self.FRRoll.update_motor_power()

        if self.useBR:
            self.BRHip.set_target_rad(joint_positions[2])
            self.BRKnee.set_target_rad(joint_positions[6])
            self.BRRoll.set_target_rad(joint_positions[10])
            
            self.BRKnee.update_motor_power()
            self.BRHip.update_motor_power()
            self.BRRoll.update_motor_power()
        
        if self.useBL:
            self.BLHip.set_target_rad(joint_positions[3])
            self.BLKnee.set_target_rad(joint_positions[7])
            self.BLRoll.set_target_rad(joint_positions[11])

            self.BLKnee.update_motor_power()
            self.BLHip.update_motor_power()
            self.BLRoll.update_motor_power()
