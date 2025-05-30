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
    


class StraighLine(FSM):
    def __init__(self, logger = None):
        super().__init__(logger)
        
    def update(self, curr_time, obstacles):
        return "forward"

class PingPong(FSM):
    previousAction = 'neutral'
    actions = ['lwr','rwr']
    state = 'none'
    def __init__(self, logger = None):
        super().__init__(logger)
        
    def update(self, cell_data):
        global previousAction
        self.log(f"Cell Data length" + str(len(cell_data)))
        range2, range1, range3 = cell_data[13], cell_data[19], cell_data[25]
        range2 /= 1000
        range1 /= 1000
        range3 /= 1000
        
        if self.previousAction == 'b':
            self.action =  self.actions[np.random.randint(0,1)]
        elif range2 < 1 and range3 < 1 and (range1 > 1 or range1 < 0):
            self.action = 'f'
        elif (range2 < 1 and range2 != 0) or (range2 < 1 and range1 < 1 and range2 != 0 and range1 != 0):
            self.action = 'rwr'
        elif (range3 < 1 and range3 != 0) or (range3 < 1 and range1 < 1 and range3 != 0 and range1 != 0):
            self.action = 'lwr'
        elif range1 < 0.3 and range1 > 0:
            # print('back hit')
            self.action = 'b'
        elif range2 < 1 and range3 < 1 and range2 > 0 and range3 > 0:
            action = 'b'
        elif range1 > 0.5 and range1 > 0:
            action = 'f'
        else:
            action = 'b'
        # else:
        #     if state == 'random':
        #         action = self.previousAction
        #     else:
        #         randInt = np.random.randint(0,1)
        #         action = actions[randInt]
        #     pass
        #     state = 'random'

        previousAction = action

        return action
        # return "forward"
    

