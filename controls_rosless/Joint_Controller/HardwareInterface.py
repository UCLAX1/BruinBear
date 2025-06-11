import struct
import can
import os
import json
import time

class ProcessEncoderData(can.Listener):
    STATUS_2 = 0x2051880
    
    def __init__(self, motor_pos):
        self.motor_pos = motor_pos
        # self.motor_init_pos = [None] * 30
        
    def on_message_received(self, msg):
        if msg.arbitration_id & 0xFFFFFFFF0 == self.STATUS_2: #only get status frame 2
            raw_position_bytes = msg.data[0:4]
            position_float = struct.unpack('<f', raw_position_bytes)[0]            
            motorId = msg.arbitration_id & 0xF
            
            # if self.motor_init_pos[motorId] is None: #resets motor position on the first read
            #     self.motor_init_pos[motorId] = position_int
            
            self.motor_pos[motorId] = position_float #- self.motor_init_pos[motorId]
            return
    
class CanBus:
    EXT_MASK = 0x80000000
     
    def __init__(self):
        self.notifier = None
        self.bus = None
        self.motor_pos = [None] * 30

    def start(self):
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
            listener = ProcessEncoderData(motor_pos=self.motor_pos)
            self.notifier = can.Notifier(self.bus, [listener])
            
            print(f"CAN bus started")
        except Exception as e:
            print(f"Failed to start CAN bus: {e}")
            self.bus = None

    def send_message(self, arbitration_id: int, data: bytes):
        if not self.bus:
            print("CAN bus is not started.")
            return
        
        arbitration_id = arbitration_id | self.EXT_MASK
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=True)
        try:
            self.bus.send(message)
            # print(f"Message sent: ID={arbitration_id}, Data={data}")
        except can.CanError as e:
            print(f"Failed to send message: {e}")
    
    def close(self):
        self.notifier.stop()
        self.bus.shutdown()
        print("CAN bus stopped.")
        
    def __del__(self):
        self.close()
            
            
class Motor:
    Duty_Cycle_ID = 0x2050080
    Heartbeat_ID = 0x2052480
    MAX_DUTY_CYCLE = 0.5 # safe threshold for now
    INIT_POS_FILE = "motor_init_pos.json"
    
    def __init__(self, can_bus: CanBus, motor_id: int, logger = None):
        self.can_bus = can_bus
        self.motor_id = motor_id
        self.logger = logger
        
        # load initial pos
        self.init_pos = 0
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)
                self.init_pos = data.get(str(self.motor_id), 0)
        
        # wait to get a motor position to ensure motor is connected via CAN
        self.log(f"waiting for motor {self.motor_id}")
        max_wait = 3
        start = time.time()
        while self.can_bus.motor_pos[self.motor_id] is None:
            if time.time() - start > max_wait:
                self.log(f"WARNING: Timeout waiting for motor {self.motor_id} position.")
                raise Exception("Motor not connected")
            time.sleep(0.05)
        
        self.log(f"motor connected {self.motor_id}")
        self.log(f"current motor position {self.get_pos()}")

    def log(self, msg):
        if self.logger:
            self.logger.info(str(msg))
        else:
            print(str(msg))
        
    def set_power(self, power: float):
        msg_id = self.Duty_Cycle_ID + self.motor_id
        
        power = max(-1, min(power, 1))  # Clamp power to 0-1 range
        power = power * self.MAX_DUTY_CYCLE # Normalize power to duty cycle range
        
        duty_data = struct.pack('<f', power)
        data = bytearray(8)
        data[:4] = duty_data
        
        self.can_bus.send_message(msg_id, data)
        
    def send_heartbeat(self):
        msg_id = self.Heartbeat_ID + self.motor_id
        heartbeat_data = [0xFF] * 8
        
        self.can_bus.send_message(msg_id, heartbeat_data)
        
    def get_pos(self):
        return self.can_bus.motor_pos[self.motor_id] - self.init_pos
    
    def reset_encoder(self):
        pos = self.can_bus.motor_pos[self.motor_id]
        self.init_pos = pos
        
        self.log(f"reseting encoders to {pos}")
            
        # update json file
        data = {}
        if os.path.exists(self.INIT_POS_FILE):
            with open(self.INIT_POS_FILE, "r") as f:
                data = json.load(f)

        data[str(self.motor_id)] = pos

        with open(self.INIT_POS_FILE, "w") as f:
            json.dump(data, f, indent=2)
        

        
        
        