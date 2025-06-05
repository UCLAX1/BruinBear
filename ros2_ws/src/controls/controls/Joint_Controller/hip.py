import time
import math
try: 
    from HardwareInterface import CanBus, Motor
except ImportError:
    from controls.Joint_Controller.HardwareInterface import CanBus, Motor

class Hip:
    kp = 0.1  # Proportional gain
    ki = 0.0  # Integral gain
    kd = 0.00  # Derivative gain
    dt = 0.01  # Time step for PID loop
    MAX_ABS_TICKS = 11
    MIN_TICKS = 0
    MAX_TICKS = 0
    MAX_POWER = 0.6
    
    def __init__(self, motor_id : int, can_bus : CanBus, inverted=False, leg_id="leg"):
        self.motor = Motor(can_bus, motor_id)
        self.leg_id = leg_id
        self.current_position = 0
        self.inverted = inverted
        
        if inverted:
            self.MIN_TICKS = 0
            self.MAX_TICKS = self.MAX_ABS_TICKS
        else:
            self.MIN_TICKS = -self.MAX_ABS_TICKS
            self.MAX_TICKS = 0
                
        # PID variables
        self.previous_error = 0
        self.integral = 0
        
        self.motor_power = 0
        self.target_position = 0
            
    def set_target_ticks(self, target):
        target = max(self.MIN_TICKS, min(self.MAX_TICKS, target))
        self.target_position = target
        
    def set_target_rad(self, target):
        #x = target
        #ticks = ((-15.279) * x) * (-1 if self.inverted else 1)
        target = abs(target)
        #ticks = 7.64 * target * (-1 if self.inverted else 1) for standing reset
        ticks = (-11.357 + 7.64 * target) * (-1 if self.inverted else 1)
        self.set_target_ticks(ticks)
        return ticks

    def get_current_ticks(self):
        return self.motor.get_pos()
    
    def update_motor_power(self):
        """Run the PID loop and calculate motor power"""
        # Read the current position
        self.current_position = self.get_current_ticks()
        
        # Calculate the error
        error = self.target_position - self.current_position
        
        print ("target", self.target_position)
        
        # PID calculations
        proportional = self.kp * error
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        
        # Compute motor power
        power = proportional + (self.ki * self.integral) + (self.kd * derivative)
        
        # Update the previous error for the next iteration
        self.previous_error = error
        
        # Simulate motor output (replace with actual motor command)
        self.apply_motor_power(power)
    
    def apply_motor_power(self, power):
        self.motor_power = max(-self.MAX_POWER, min(self.MAX_POWER, power))
        print ('setting power: ', self.motor_power)
        self.motor.set_power(self.motor_power)
        self.motor.send_heartbeat()
        
    def reset_encoder(self):
        self.motor.reset_encoder()
    
    def __str__(self):
        """Return a string representation of the hip state."""
        return (f"Hip[{self.leg_id}]: Position={self.current_position:.2f}, "
                f"Target={self.target_position}, Power={self.motor_power:.2f}")
