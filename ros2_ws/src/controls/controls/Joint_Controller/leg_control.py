from hip import Hip
from knees import Knee
import time

class LegController:
    def __init__(self, dt=0.01):
        # Create 4 Hip joints
        self.hips = [
            Hip(kp=1.0, ki=1.0, kd=1.0, leg_id="front-left-hip", dt=dt),
            Hip(kp=1.0, ki=1.0, kd=1.0, leg_id="front-right-hip", dt=dt),
            Hip(kp=1.0, ki=1.0, kd=1.0, leg_id="back-left-hip", dt=dt),
            Hip(kp=1.0, ki=1.0, kd=1.0, leg_id="back-right-hip", dt=dt),
        ]
        
        # Create 4 Knee joints
        self.knees = [
            Knee(kp=1.0, ki=1.0, kd=1.0, leg_id="front-left-knee", dt=dt),
            Knee(kp=1.0, ki=1.0, kd=1.0, leg_id="front-right-knee", dt=dt),
            Knee(kp=1.0, ki=1.0, kd=1.0, leg_id="back-left-knee", dt=dt),
            Knee(kp=1.0, ki=1.0, kd=1.0, leg_id="back-right-knee", dt=dt),
        ]
        self.dt = dt
    
    def update_target_positions(self, hip_targets, knee_targets):
        """Update target positions for all hips and knees"""
        for hip, target in zip(self.hips, hip_targets):
            hip.set_target_position(target)
        for knee, target in zip(self.knees, knee_targets):
            knee.set_target_position(target)
    
    def run(self):
        """Continuously run the PID controllers for all joints"""
        while True:
            # Update motor power for all joints
            for hip in self.hips:
                hip.update_motor_power()
            for knee in self.knees:
                knee.update_motor_power()
            
            # Print the status of each joint
            print("\nJoint Status:")
            for hip in self.hips:
                print(hip)
            for knee in self.knees:
                print(knee)
            
            time.sleep(self.dt)

