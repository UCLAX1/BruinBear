import numpy as np
class FSM():
    def __init__(self, logger = None):
        self.logger = logger
        
    def update(self, curr_time, obstacles):
        return 'f'
        
    def log(self, msg):
        if self.logger:
            self.logger.info(str(msg))


class StraighLine(FSM):
    def __init__(self, logger = None):
        super().__init__(logger)
        
    def update(self, curr_time, obstacles):
        return "forward"

class PingPong(FSM):
    previousAction = 'neutral'
    actions = ['left','right']
    state = 'none'
    def __init__(self, logger = None):
        super().__init__(logger)
        
    def update(self, curr_time, cell_data, actions):
        range2, range1, range3 = cell_data

        if self.previousAction == 'back':
            self.action =  self.actions[np.random.randint(0,1)]
        elif range2 < 1 and range3 < 1 and (range1 > 1 or range1 < 0):
            self.action = 'fwd'
        elif (range2 < 0.5 and range2 != -1) or (range2 < 1 and range1 < 1 and range2 != -1 and range1 != -1):
            self.action = 'right'
        elif (range3 < 0.5 and range3 != -1) or (range3 < 1 and range1 < 1 and range3 != -1 and range1 != -1):
            self.action = 'left'
        elif range1 < 0.3 and range1 > 0:
            # print('back hit')
            self.action = 'back'
        elif range2 < 1 and range3 < 1 and range2 > 0 and range3 > 0:
            action = 'back'
        elif range1 > 0.5 and range1 > 0:
            action = 'fwd'
        else:
            if state == 'random':
                action = self.previousAction
            else:
                randInt = np.random.randint(0,1)
                action = actions[randInt]
            pass
            state = 'random'

        previousAction = action

        return action
        # return "forward"
    

