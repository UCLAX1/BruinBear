import sys
import os
import serial
import cv2
import time

import serial.tools.list_ports

ports = serial.tools.list_ports.comports()
for p in ports:
    print(p.device)
    

# Add the face_detection directory to Python path
sys.path.append(os.path.join(os.path.dirname(__file__), '../../face_dectection'))
from faceTracking import FaceDetector

# CONFIGURATION
# SERIAL_PORT = "/dev/ttyACM0"  # Update if different
SERIAL_PORT = "/dev/ttyUSB0"
BAUD_RATE = 9600

LEFT_SERVO = [60, 1]
RIGHT_SERVO = [120, 180]
FRAME_DIM = [-0.5, 0.5]

# CALCULATED CONSTANTS
LEFT_SERVO_DISTANCE = abs(LEFT_SERVO[1] - LEFT_SERVO[0])
LEFT_SERVO_MIDDLE = (LEFT_SERVO[1] + LEFT_SERVO[0]) / 2
RIGHT_SERVO_DISTANCE = abs(RIGHT_SERVO[1] - RIGHT_SERVO[0])
RIGHT_SERVO_MIDDLE = (RIGHT_SERVO[1] + RIGHT_SERVO[0]) / 2
FRAME_DIM_DISTANCE = FRAME_DIM[1] - FRAME_DIM[0]
FRAME_DIM_MIDDLE = (FRAME_DIM[1] + FRAME_DIM[0]) / 2



class HeadSerialController:
    
    def __init__(self, port=SERIAL_PORT, baudrate=BAUD_RATE, timeout=1):
        self.average = 0.5
        self.difference = 0
        self.const = 0.1
        
        try:
            self.arduino = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)  # open serial port
            print(self.arduino.name)         # check which port was really used
            time.sleep(2)
        except Exception as e:
            print(f"Could not connect to Arduino: {e}")
            self.arduino = None
    
    def send_command(self, coordinates):
        msg = self.convert_coords(coordinates)
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

        self.average = -ycoord * self.const + self.average
        self.average = min(max(self.average, 0), 1)  # Clamp average to frame dimensions

        self.difference = -xcoord * self.const + self.difference
        self.difference = min(max(self.difference, -1), 1)  # Clamp difference to frame dimensions

        # print(f"X-Coord: {xcoord}, Y-Coord: {ycoord}, Average: {self.average}, Difference: {self.difference}")

        lpos = self.average - self.difference/2 # between 0 and 1
        rpos = self.average + self.difference/2 # between 0 and 1

        lpos = min(max(lpos, 0), 1)  # Clamp to [0, 1]
        rpos = min(max(rpos, 0), 1)  # Clamp to [0, 1]

        ltarget = LEFT_SERVO[1] + LEFT_SERVO_DISTANCE * lpos
        rtarget = RIGHT_SERVO[1] - RIGHT_SERVO_DISTANCE * rpos
        
        # Convert to integers and clamp to servo range (0-180)
        left_servo = int(ltarget)
        right_servo = int(rtarget)

        # Clamp servo values to valid ranges
        left_servo = min(LEFT_SERVO[0], max(left_servo, LEFT_SERVO[1]))
        right_servo = max(RIGHT_SERVO[0], min(right_servo, RIGHT_SERVO[1]))

        print(f"Left Servo: {left_servo}, Right Servo: {right_servo}")
        
        # Send as 2 bytes
        message = bytes([left_servo, right_servo])

        return message
        
def runCamera():
    """Run camera, detect faces, calculate coordinates and send commands"""
    # Initialize face detector
    cap = cv2.VideoCapture("/dev/video0")
    pTime = 0
    detector = FaceDetector()
    headSerialController = HeadSerialController()
    
    while True:
        success, img = cap.read()
        img, bboxs = detector.findFaces(img)
        #print(bboxs)
        center = detector.returnCenter(img)
        #print(coordinates)
        relative = detector.returnRelativePosition(center, img)

        if relative is not None:

            #print(relative)
            returnSignal = detector.returnSignal(relative)

            returnSignal = tuple(int(x * 100) / 100 for x in returnSignal)

            print(returnSignal)

            headSerialController.send_command(returnSignal)


        
        cTime = time.time()
        fps = 1 / (cTime - pTime)
        pTime = cTime
        cv2.putText(img, f'FPS: {int(fps)}', (20, 70), cv2.FONT_HERSHEY_PLAIN, 3, (0, 255, 0), 2)
        cv2.imshow("Image", img)
        cv2.waitKey(1)


def find_arduino_port(baudrate=9600, timeout=1):
    ports = serial.tools.list_ports.comports()
    for port in ports:
        device = port.device
        print(f"Trying port: {device}")
        try:
            ser = serial.Serial(device, baudrate=baudrate, timeout=timeout)
            time.sleep(0.5)  # wait for Arduino reset after opening port

            # Optional: send a command or read some data to confirm it's Arduino
            # For example, read a line if Arduino sends data on startup
            if ser.in_waiting:
                response = ser.readline().decode(errors='ignore').strip()
                print(f"Received: {response}")
                # Put your own check here if you know expected response

            # If open succeeds, this port is likely the Arduino
            ser.close()
            print(f"Found device on port: {device}")
            return device

        except (serial.SerialException, OSError) as e:
            print(f"Failed on port {device}: {e}")
            continue

    return None

if __name__ == "__main__":
    runCamera()
    # headSerialControllerNew = HeadSerialController()
    # while True:
    #     headSerialControllerNew.send_command((0.5, 0.5))
    # arduino_port = find_arduino_port()
    # if arduino_port:
    #     print(f"Arduino found on port {arduino_port}")
    # else:
    #     print("Arduino not found.")