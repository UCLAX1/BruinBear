import wpilib
import rev

class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        # Initialize Spark MAX controllers for each leg
        self.leg_motors = [
            rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless),  # Front Left
            rev.CANSparkMax(2, rev.CANSparkMax.MotorType.kBrushless),  # Front Right
            rev.CANSparkMax(3, rev.CANSparkMax.MotorType.kBrushless),  # Rear Left
            rev.CANSparkMax(4, rev.CANSparkMax.MotorType.kBrushless),  # Rear Right
        ]

        # Gait parameters
        self.gait_speed = 0.5  # Speed of leg movement
        self.gait_phase = 0  # Current phase of the gait cycle

    #For testing motors together
    #Change the arguments to set function as needed
    def teleopPeriodic(self):
        # Alternating gait: Move front left and rear right together, then front right and rear left
        if self.gait_phase == 0:
            # Move front left and rear right legs forward
            self.leg_motors[0].set(self.gait_speed)  # Front Left
            self.leg_motors[3].set(self.gait_speed)  # Rear Right
            # Move front right and rear left legs backward
            self.leg_motors[1].set(-self.gait_speed)  # Front Right
            self.leg_motors[2].set(-self.gait_speed)  # Rear Left
        else:
            # Move front right and rear left legs forward
            self.leg_motors[1].set(self.gait_speed)  # Front Right
            self.leg_motors[2].set(self.gait_speed)  # Rear Left
            # Move front left and rear right legs backward
            self.leg_motors[0].set(-self.gait_speed)  # Front Left
            self.leg_motors[3].set(-self.gait_speed)  # Rear Right

        # Switch gait phase
        self.gait_phase = 1 - self.gait_phase

    def disabledInit(self):
        # Stop all motors when the robot is disabled
        for motor in self.leg_motors:
            motor.set(0)

if __name__ == "__main__":
    wpilib.run(MyRobot)