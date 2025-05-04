import navigation_logic

fwd_buffer = 500
action_buffer = 2000
counter = 0
previous_nav = 'none'
nav = 'non'


def automate(ranges: list,robot_fsm):
    '''3 rangefinder distances + robot_fsm: \n
    ([range1, range2, range3], fsm)'''
    global fwd_buffer,action_buffer,counter, previous_nav, nav
    
    # print(str(min(ranges)))
    
    if min(ranges) < 0.2 and min(ranges) != -1:
        nav = 'back'
    elif previous_nav == 'fwd':
        if counter >= fwd_buffer or counter == 0:
            nav = navigation_logic.navigate(ranges[0], ranges[1], ranges[2])
            counter = 0
    elif counter >= action_buffer or counter == 0:
        nav = navigation_logic.navigate(ranges[0], ranges[1], ranges[2])
        counter = 0
    counter += 1
    # print(f'nav: {nav}')
    # print(f'robot state: {robot_fsm.state}')
    if nav != previous_nav:
        robot_fsm.neutralPos()
    elif nav == 'fwd' :
        robot_fsm.step(1)
    elif nav == 'back':
        robot_fsm.step(-1)
    elif nav == 'left':
        robot_fsm.turn('left')
    elif nav == 'right':
        robot_fsm.turn('right')
    if robot_fsm.state == 'INIT':
        previous_nav = nav