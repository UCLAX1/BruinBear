import sys
import os
import serial
import cv2
import time

# Add the face_detection directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../face_dectection'))
from face_detection import FaceDetector

# CONFIGURATION
SERIAL_PORT = "/dev/ttyACM0"  # Update if different
BAUD_RATE = 9600

LEFT_SERVO = [60, 1]
RIGHT_SERVO = [120, 180]
FRAME_DIM = [-0.5, 0.5]

# CALCULATED CONSTANTS
LEFT_SERVO_DISTANCE = LEFT_SERVO[1] - LEFT_SERVO[0]
LEFT_SERVO_MIDDLE = (LEFT_SERVO[1] + LEFT_SERVO[0]) / 2
RIGHT_SERVO_DISTANCE = RIGHT_SERVO[1] - RIGHT_SERVO[0]
RIGHT_SERVO_MIDDLE = (RIGHT_SERVO[1] + RIGHT_SERVO[0]) / 2
FRAME_DIM_DISTANCE = FRAME_DIM[1] - FRAME_DIM[0]
FRAME_DIM_MIDDLE = (FRAME_DIM[1] + FRAME_DIM[0]) / 2



class HeadSerialController:
    
    def __init__(self, port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1):
        self.average = 0.5
        self.difference = 0
        self.const = 0.1
        
        try:
            self.arduino = serial.Serial(port, baudrate, timeout)  # open serial port
            print(self.arduino.name)         # check which port was really used
            time.sleep(2)
        except Exception as e:
            print(f"Could not connect to Arduino: {e}")
            self.arduino = None
    
    def send_command(self, coordinates):
        if self.arduino:
            try:
                message = self.convert_coords(coordinates)
                self.arduino.write(message)
                print(f"Sent: {message}")
            except Exception as e:
                print(f"Error sending command: {e}")
    
    def convert_coords(self, coordinates):
        xcoord = coordinates[0]
        ycoord = coordinates[1]
        self.average = ycoord * self.const + self.average
        self.difference = xcoord * self.const + self.difference

        lpos = self.average - self.difference/2 
        rpos = self.average + self.difference/2

        ltarget = LEFT_SERVO[0] - LEFT_SERVO_DISTANCE * lpos
        rtarget = RIGHT_SERVO[0] + RIGHT_SERVO_DISTANCE * rpos
        
        
        # Convert to integers and clamp to servo range (0-180)
        left_servo = int(ltarget)
        right_servo = int(rtarget)

        # Clamp servo values to valid ranges
        left_servo = min(LEFT_SERVO[0], max(left_servo, LEFT_SERVO[1]))
        right_servo = max(RIGHT_SERVO[0], min(right_servo, RIGHT_SERVO[1]))
        
        # Send as 2 bytes
        message = bytes([left_servo, right_servo])

        return message
        
    def runCamera(self):
        """Run camera, detect faces, calculate coordinates and send commands"""
        # Initialize face detector
        detector = FaceDetector()
        
        