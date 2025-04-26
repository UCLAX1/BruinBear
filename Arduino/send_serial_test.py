# send the serial messages to the arduino

import serial
import time
import sys
import tty
import termios

# CONFIGURATION
SERIAL_PORT = "/dev/ttyACM0"  # Update if different
BAUD_RATE = 9600

# Open serial connection
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

print("Press Up or Down arrow to send commands. Ctrl+C to exit.")

def get_key():
    """Wait for keypress and return character."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(3)  # read 3 chars for arrow keys
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

while True:
    key = get_key()
    if key == '\x1b[A':  # Up arrow
        print("Sending: ON")
        ser.write(b'ON\n')
    elif key == '\x1b[B':  # Down arrow
        print("Sending: OFF")
        ser.write(b'OFF\n')
    else:
        print("Exiting...")
        ser.close()
        break
