

from geometry_msgs.msg import Twist
import json
import ast

def calc_red_ball_coordinate(area, own_coordinate):

    # Linear approximation :hard coding
    distance = -1.1492 * area + 1891.2



def rotation_operate(command):
    # back ward
    if command == 0:
        x = -0.1
        th = 0
    # turn right
    elif command == 1:
        x = 0
        th = -0.5
    # turn left
    elif command == 2:
        x = 0
        th = 0.5
    # forward
    elif command == 3:
        x = 0.1
        th = 0.0
    elif command == 99:
        x = 0
        th = 0
    twist = Twist()
    twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
    return twist


def check_score():
    pass


def check_possession_marker(war_state):
    obtain_targets_list = []
    targets_list = []
    if war_state is None:
        return obtain_targets_list
    #print(dir(war_state.data))
    #print(war_state.data) 
    json_war_state = json.loads(war_state.data)
    for key in json_war_state:
        if key == 'scores':
            # TODO
            pass
        elif key == 'targets':
            targets_list = json_war_state[key]
            break
    for target in targets_list:
        for key in target:
            if key == 'player' and target[key] == 'r':
                obtain_targets_list.append(target['name'])
    return obtain_targets_list
        