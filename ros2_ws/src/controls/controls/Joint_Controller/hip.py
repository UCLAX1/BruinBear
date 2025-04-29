import time
from HardwareInterface import CanBus, Motor

class Hip:
    kp = 0.00001  # Proportional gain
    ki = 0.0  # Integral gain
    kd = 0.00000005  # Derivative gain
    dt = 0.01  # Time step for PID loop
    
    def __init__(self, motor_id : int, can_bus : CanBus, inverted=False, leg_id="leg"):
        self.motor = Motor(can_bus, motor_id)
                
        # PID variables
        self.previous_error = 0
        self.integral = 0
        
        self.motor_power = 0
        self.target_position = 0
            
    def set_target_position(self, target):
        self.target_position = target
    
    def get_current_position(self):
        return self.motor.get_pos()
    
    def update_motor_power(self):
        """Run the PID loop and calculate motor power"""
        # Read the current position
        self.current_position = self.get_current_position()
        
        # Calculate the error
        error = self.target_position - self.current_position
        
        # PID calculations
        proportional = self.kp * error
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        
        # Compute motor power
        self.motor_power = proportional + (self.ki * self.integral) + (self.kd * derivative)
        
        # Update the previous error for the next iteration
        self.previous_error = error
        
        # Simulate motor output (replace with actual motor command)
        self.apply_motor_power(self.motor_power)
    
    def apply_motor_power(self, power):
        self.motor.set_power(power)
        self.motor.send_hearbeat()
    
    def __str__(self):
        """Return a string representation of the hip state."""
        return (f"Hip[{self.leg_id}]: Position={self.current_position:.2f}, "
                f"Target={self.target_position}, Power={self.motor_power:.2f}")
