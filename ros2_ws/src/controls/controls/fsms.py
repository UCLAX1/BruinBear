import numpy as np

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

class FSM():
    def __init__(self, logger = None):
        self.logger = logger
        
    def update(self, curr_time, obstacles):
        return 'f'
        
    def log(self, msg):
        if self.logger:
            self.logger.info(str(msg))

    def subscriber(self, Node):
       self.subscription = self.create_subscription(Float32MultiArray, 'position_data', self.fsm_callback, 10)
    


class StraightLine(FSM):
    def __init__(self, logger = None):
        super().__init__(logger)
        
    def update(self, curr_time, obstacles):
        return "forward"

class PingPong(FSM):
    void_range = 10
    mid_range = 1
    emergency_range = 0.3
    close_range = 0.5
    far_range = 5
    
    previousAction = 's'
    actions = ['lwr','rwr']
    action = ''
    # state = 'none'
    def __init__(self, logger = None):
        super().__init__(logger)
        
    def update(self, cell_data):
        # self.log(f"Cell Data length" + str(len(cell_data)))
        left_range, middle_range, right_range = cell_data[1], cell_data[19], cell_data[24]
        left_range /= 1000
        middle_range /= 1000
        right_range /= 1000
        self.log(f"Range Middle {str(middle_range)}")
        self.log(f'Range Left {str(left_range)}')
        self.log(f'Range Right: {str(right_range)}')
        
        if self.previousAction == 'b':
            self.action = self.actions[np.random.randint(0,2)]
        elif middle_range > self.void_range or left_range > self.void_range or right_range > self.void_range:
            self.action = 'b'
        elif middle_range < self.emergency_range and middle_range > 0:
            # print('back hit')
            self.action = 'b'
        elif (left_range < self.mid_range and right_range < self.mid_range and left_range > 0 and right_range > 0) or (middle_range > self.void_range and left_range > self.void_range and right_range > self.void_range):
            self.action = 'b'
        elif middle_range > self.close_range and middle_range > 0:
            self.action = 'f'
        elif left_range < self.mid_range and right_range < self.mid_range and (middle_range > self.mid_range or middle_range < 0):
            self.action = 'f'
        elif (left_range < self.mid_range and left_range != 0 and left_range < self.far_range) or (left_range < self.mid_range and middle_range < self.mid_range and left_range != 0 and middle_range != 0):
            self.action = 'rwr'
        elif (right_range < self.mid_range and right_range != 0 and right_range < self.far_range) or (right_range < self.mid_range and middle_range < self.mid_range and right_range != 0 and middle_range != 0):
            self.action = 'lwr'
        else:
            self.action = 'b'
      
        # else:
        #     if state == 'random':
        #         action = self.previousAction
        #     else:
        #         randInt = np.random.randint(0,1)
        #         action = actions[randInt]
        #     pass
        #     state = 'random'

        self.previousAction = self.action

        return self.action  
        # return "forward"
    

