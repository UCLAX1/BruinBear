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
    def __init__(self, logger = None):
        super().__init__(logger)
        
    def update(self, curr_time, obstacles):
        return "forward"
    

