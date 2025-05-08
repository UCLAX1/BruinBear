import numpy as np
previousAction = 'neutral'
actions = ['left', 'right']
state = 'none'

def navigate(range1,range2,range3):
    print(f'range1: {range1} range2: {range2} range3: {range3}')
    global previousAction, actions, state
    action = 'neutral'
    if previousAction == 'back':
       action =  actions[np.random.randint(0,1)]
    elif range2 < 1 and range3 < 1 and (range1 > 1 or range1 < 0):
        action = 'fwd'
    elif (range2 < 0.5 and range2 != -1) or (range2 < 1 and range1 < 1 and range2 != -1 and range1 != -1):
        action = 'right'
    elif (range3 < 0.5 and range3 != -1) or (range3 < 1 and range1 < 1 and range3 != -1 and range1 != -1):
        action = 'left'
    elif range1 < 0.3 and range1 > 0:
        # print('back hit')
        action = 'back'
    elif range2 < 1 and range3 < 1 and range2 > 0 and range3 > 0:
        action = 'back'
    elif range1 > 0.5 and range1 > 0:
        action = 'fwd'
    else:
        if state == 'random':
            action = previousAction
        else:
            randInt = np.random.randint(0,1)
            action = actions[randInt]
        pass
        state = 'random'

    previousAction = action

    return action