import can
import time

# enum control_mode {
Duty_Cycle_Set = 0x2050080
Speed_Set = 0x2050480
Smart_Velocity_Set = 0x20504C0
Position_Set = 0x2050C80
Voltage_Set = 0x2051080
Current_Set = 0x20510C0
Smart_Motion_Set = 0x2051480
Position_Set = 0x2050C80
Heartbeat_Set = 0x2052C80

CAN_EFF_FLAG = 0x80000000


# enum status_frame_id {
status_0 = 0x2051800
status_1 = 0x2051840
status_2 = 0x2051880
status_3 = 0x20518C0
status_4 = 0x2051900

bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)

device_id = 1

# // Heartbeat Frame
HEARTBEAT_ID = 0x2052C80
HEARTBEAT_DATA = [ 255, 255, 255, 255, 255, 255, 255, 255 ]
HEARTBEAT_SIZE = 8;

# // Control Frame
CONTROL_SIZE = 8;

# // Status Frame
STATUS_SIZE = 2;

print("CAN CONTROLLER");

#   // Setup predefined CAN Messages. ALL MESSAGES MUST BE INITIALIZED IN SETUP
#   // Heartbeat
# heartbeat_frame.can_id = HEARTBEAT_ID | CAN_EFF_FLAG;
# heartbeat_frame.can_dlc = HEARTBEAT_SIZE;
# pack_data(heartbeat_frame, HEARTBEAT_DATA, HEARTBEAT_SIZE);

#   // Status
# status_frame.can_dlc = STATUS_SIZE;

#   // Synchronize all REV motor controllers
#   // Not doing so may be causing txrx errors
#   // send_CAN_packet(0x20524c0);

#   Process for updateing period of periodic status frames
#   set_status_frame_period(11, status_1, 100);


import struct

# CAN Bus Setup
bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)

def send_speed_set(device_id, speed_rpm):
    """Send Speed Set command to motor controller with correct Extended ID"""
    
    # Compute the extended ID correctly
    ext_id = (Smart_Velocity_Set + device_id) | 0x80000000  # Ensure Extended Frame Format

    # Convert speed setpoint to IEEE 754 floating point (4 bytes, little-endian)
    speed_data = struct.pack('<f', speed_rpm)

    # Construct CAN Data (8 bytes)
    data = bytearray(8)
    data[:4] = speed_data  # Target Speed (bytes 0-3)
    data[4] = 0  # Aux[0]
    data[5] = 0  # Aux[1]
    data[6] = 0  # Reserved
    data[7] = 0  # Reserved

    # Create and send CAN message
    msg = can.Message(arbitration_id=ext_id, is_extended_id=True, data=data)
    
    try:
        bus.send(msg)
        print(f"Sent Speed Set command (ID: {hex(ext_id)}, Speed: {speed_rpm} RPM)")
        print ("control data sent:", msg)
    except can.CanError as e:
        print("Error sending Speed Set command:", e)
        
def send_heartbeat(device_id):
    heartbeat_id = (Heartbeat_Set + device_id) | 0x80000000
    heartbeat_data = [0xFF] * 8
    msg = can.Message(arbitration_id=heartbeat_id, is_extended_id=True, data=heartbeat_data)
    bus.send(msg)
    print(f"Sent Heartbeat to device {device_id} with ID {hex(heartbeat_id)}")
    print ("heartbeat data sent:", msg)
    
def send_device_specific_heartbeat(device_id):
    """Sends the special 'internal heartbeat' frame that makes the motor actually move"""
    internal_heartbeat_id = (0x2052480 + device_id) | 0x80000000
    data = [0xFF] * 8
    msg = can.Message(arbitration_id=internal_heartbeat_id, is_extended_id=True, data=data)
    bus.send(msg)
    print(f"Sent internal heartbeat to device {device_id} (ID: {hex(internal_heartbeat_id)})")
    
    
def send_duty_cycle(device_id, percent_output):
    ext_id = (Duty_Cycle_Set + device_id) | 0x80000000
    duty_data = struct.pack('<f', percent_output)  # e.g., 0.5 for 50%
    data = bytearray(8)
    data[:4] = duty_data
    msg = can.Message(arbitration_id=ext_id, is_extended_id=True, data=data)
    bus.send(msg)
    print(f"Sent Duty Cycle command: {percent_output * 100:.1f}%")
    
    
def send_position_set(device_id, position_rotations):
    """Send Position Set command to SPARK MAX"""
    ext_id = (Position_Set + device_id) | 0x80000000

    # Pack position (float) as little-endian
    pos_data = struct.pack('<f', position_rotations)

    # Build 8-byte payload
    data = bytearray(8)
    data[:4] = pos_data       # Position (rotations)
    data[4] = 0               # Aux 0
    data[5] = 0               # Aux 1
    data[6] = 0               # Reserved
    data[7] = 0               # Reserved

    msg = can.Message(arbitration_id=ext_id, is_extended_id=True, data=data)
    try:
        bus.send(msg)
        print(f"Sent Position Set command (ID: {hex(ext_id)}, Position: {position_rotations} rotations)")
    except can.CanError as e:
        print("Error sending Position Set command:", e)
        

def get_encoder_position(device_id, timeout=1.0):
    """Waits for a status 2 frame and extracts encoder position (in rotations)"""
    status_2_id = (0x2051880 + device_id)

    # Listen for messages on the bus
    print(f"Waiting for position frame from device {device_id}...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        msg = bus.recv(timeout=timeout)
        if msg is None:
            continue
        if msg.arbitration_id == status_2_id and msg.is_extended_id:
            # Extract position from first 4 bytes
            position_bytes = msg.data[:4]
            position = struct.unpack('<f', position_bytes)[0]
            print(f"Got position: {position:.4f} rotations")
            return position

    print("Timed out waiting for position frame.")
    return None

def set_status_frame_period(device_id, frame_base_id, period_ms):
    """
    Set the transmission period of a given status frame.
    `frame_base_id`: one of status_0, status_1, ..., status_4
    `period_ms`: how often to send, in milliseconds
    """
    ext_id = (frame_base_id + device_id) | CAN_EFF_FLAG
    # data = struct.pack('<H', period_ms)
    data = [0x14, 0x00]
    msg = can.Message(arbitration_id=ext_id, is_extended_id=True, data=data)
    try:
        bus.send(msg)
        print(f"Requested Status Frame {hex(ext_id)} at {period_ms}ms")
    except can.CanError as e:
        print("Error sending status frame config:", e)

def read_can_once():
    """
    Reads and prints one CAN frame if available (non-blocking).
    Call this inside your main loop.
    """
    msg = bus.recv(timeout=0.001)  # Small timeout so it doesn’t block
    if msg is not None:
        print(f"CAN Received | ID: {hex(msg.arbitration_id)}  "
              f"{'EXT' if msg.is_extended_id else 'STD'}  "
              f"DL: {msg.dlc}  Data: {' '.join(f'{b:02X}' for b in msg.data)}")


# Example Usage: Set motor with device ID 11 to 500 RPM

# send_position_set(1, 1.0) 

time.sleep(2)
send_position_set(1, 1.0)
set_status_frame_period(1, status_2, 20)
set_status_frame_period(1, status_3, 20)
set_status_frame_period(1, status_4, 20)


while True:
    # send_speed_set(1, 1000.0)
    # send_duty_cycle(1, 0.3) 
    # send_position_set(1, 1.0) 
    read_can_once()
    send_device_specific_heartbeat(1)
    time.sleep(0.02)  # Send every second
    
# 
# send_heartbeat(1)
# time.sleep(10)