from calendar import THURSDAY
from stat import SF_APPEND
import string
from wsgiref.simple_server import demo_app
import rospy
import enum
import yaml
import os
from time import sleep
from std_msgs.msg import UInt8MultiArray

class WayneActions(enum.Enum):
    inflate = 0
    deflate = 1
    inflate_all = 2
    deflate_all = 3
    inflate_all_but = 4
    deflate_all_but = 5
    hold = 6
    inflate_xs_deflate_ys = 7

VALVES_PINS = {
    "inflate":0,
    "deflate":1,
    "flp": 2, # front left parapodia
    "frp":4,  # front right parapodis
    "mp":6,   # middle parapodia
    "blp":8,  # back left parapodia
    "brp":10, # back right parapodia
    "ld":12,  # left directional
    "rd":14   # right directional
}


def reset_valves_list():
    return [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

def inflate(**kwargs):
    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['inflate']] = 1
    for a in kwargs['inflate']:
        new_valves_list[VALVES_PINS[a]] = 1
    return new_valves_list

def inflate_all():
    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['inflate']] = 1
    for a in range(2,16,2):
        new_valves_list[a] = 1
    return new_valves_list

def inflate_all_but(**kwargs):
    new_valves_list = inflate_all()
    for a in kwargs['all_but']:
        new_valves_list[VALVES_PINS[a]] = 0
    return new_valves_list

def deflate(**kwargs):
    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['deflate']] = 1
    for a in kwargs['deflate']:
        new_valves_list[VALVES_PINS[a]+1] = 1
    return new_valves_list

def deflate_all():
    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['deflate']] = 1
    for a in range(3,16,2):
        new_valves_list[a] = 1
    return new_valves_list

def deflate_all_but(**kwargs):
    new_valves_list = deflate_all()
    for a in kwargs['all_but']:
        new_valves_list[VALVES_PINS[a]+1] = 0
    return new_valves_list

def hold():

    new_valves_list = reset_valves_list()
  
    return new_valves_list

def inflate_xs_deflate_ys(**kwargs):

    new_valves_list = reset_valves_list()
    new_valves_list[VALVES_PINS['inflate']] = 1
    new_valves_list[VALVES_PINS['deflate']] = 1

    for a in kwargs['inflate']:
        new_valves_list[VALVES_PINS[a]] = 1
   
    for a in kwargs['deflate']:
        new_valves_list[VALVES_PINS[a] + 1] = 1
  
    return new_valves_list


def publish(publisher: rospy.Publisher, data):
    msg = UInt8MultiArray()
    msg.data = data
    publisher.publish(msg)


def reset(publisher: rospy.Publisher):
    msg = UInt8MultiArray()
    msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    publisher.publish(msg)


def wait_for_sub(pub):
    while not pub.get_num_connections() > 0:
        rospy.loginfo('Waiting for RegisterState subscriber')
        rospy.sleep(1)
    
    rospy.loginfo('Starting sequence..')
    rospy.sleep(1)

def run_action(pub, action : int, sec: float, **kwargs):
    '''
    Run action on the Robot

    Parameters
    ----------
    pub: 
        Action publisher
    action : int
        A number representing one of the WayneAction enum
    sec : float
        Time (seconds) to continue to run the action
    *args: string
        The Robot parts to activate with the action, see VALVES_PINS dictionary keys
    '''

    try:
        if action == WayneActions.inflate:
            a = inflate(**kwargs)
            # print(' '*12 + f'Inflate: {a}')
        elif action == WayneActions.deflate:
            a = deflate(**kwargs)
            # print(' '*12 + f'Deflate: {a}')
        elif action == WayneActions.inflate_all:
            a = inflate_all(**kwargs)
        elif action == WayneActions.deflate_all:
            a = deflate_all(**kwargs)
        elif action == WayneActions.inflate_all_but:
            a = inflate_all_but(**kwargs)
        elif action == WayneActions.deflate_all_but:
            a = deflate_all_but(**kwargs)
        # new actions
        elif action == WayneActions.hold:
            # check this is correct way to sleep / hold ?
            a = hold()
        elif action == WayneActions.inflate_xs_deflate_ys:
            a = inflate_xs_deflate_ys(**kwargs)
            # print(f'Inflate and Deflate: {a}')
        else:
            return
    except Exception as e:
        # dictionary key error caused over inflation of one limb 
        print (f'WARNING\nException occured when trying to run action. Deflating all for 10seconds.\nDetails below: {e}')
        a = deflate_all()
        sec = 10.0
    
    publish(pub, a)
    rospy.sleep(sec)

def main():
    rospy.init_node('test_wayne_board', anonymous=True)
    pub = rospy.Publisher('register_state', UInt8MultiArray, queue_size=10)
    rate = rospy.Rate(1)

    wait_for_sub(pub)
    # DO ACTION HERE
    # Actuators: flp frp mp blp brp ld rd

    walk = True
    # wiggle = False
    # crawl = False
    # floaty = False

    # if (walk + wiggle + crawl + floaty) > 1:
    #     reset(pub)
    #     print("WARNING: more than one movement sequence selected.")
    #     return

    if walk:
        run_action(pub, WayneActions.deflate_all, 2.0)

        # run_action(pub, WayneActions.inflate, 0.2, inflate=['flp', 'frp'])
        # run_action(pub, WayneActions.inflate, 0.2, inflate=['mp'])
        run_action(pub, WayneActions.inflate, 0.1, inflate=['flp', 'frp'])
        run_action(pub, WayneActions.inflate, 0.1, inflate=['mp'])

        run_action(pub, WayneActions.deflate_all_but, 0.75, all_but=['mp', 'ld', 'rd'])

        i = 0
        while (i < 6):
            i += 1
            # maybe reduce deflates in the morning
            run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.11, inflate=['blp', 'brp'], deflate=['flp', 'frp'])
            run_action(pub, WayneActions.deflate, 1.5, deflate=['flp', 'frp'])
            run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.15, inflate=['flp', 'frp'], deflate=['mp'])
            run_action(pub, WayneActions.deflate, 1.5, deflate=['mp'])
            run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.1, inflate=['mp'], deflate=['blp', 'brp'])
            # run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.225, inflate=['mp'], deflate=['blp', 'brp'])
            run_action(pub, WayneActions.deflate, 1.0, deflate=['blp', 'brp'])

            if (i % 6 == 0):
                run_action(pub, WayneActions.deflate_all, 2.0)

        run_action(pub, WayneActions.deflate_all, 2.0)
        reset(pub)

    # if wiggle:
    #     # its a wiggly party
    #     run_action(pub, WayneActions.inflate, 0.08, inflate=['flp', 'frp', 'mp', 'blp', 'brp'])

    #     run_action(pub, WayneActions.inflate, 0.175, inflate=['ld'])
    #     i = 0
    #     while (i < 4):
    #         run_action(pub, WayneActions.deflate, 1.2, deflate=['ld'])
    #         run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.175, inflate=['rd'], deflate=['ld'])
    #         run_action(pub, WayneActions.deflate, 1.2, deflate=['rd'])
    #         run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.175, inflate=['ld'], deflate=['rd'])
    #         i += 1
    #     run_action(pub, WayneActions.deflate, 1.2, deflate=['ld'])
    #     run_action(pub, WayneActions.deflate_all, 2.0)
    #     reset(pub)

    # if crawl:
    #     run_action(pub, WayneActions.inflate, 0.2, inflate=['ld', 'frp', 'brp'])
    #     i = 0
    #     while (i < 1):
    #         run_action(pub , WayneActions.deflate, 1.2, deflate=['ld', 'frp', 'brp'])
    #         run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.2, inflate=['rd', 'flp', 'blp'], deflate=['ld', 'frp', 'brp'])
    #         run_action(pub , WayneActions.deflate, 1.2, deflate=['rd', 'flp', 'blp'])
    #         run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.2, inflate=['ld', 'frp', 'brp'], deflate=['rd', 'flp', 'blp'])
    #         i += 1
    #     run_action(pub, WayneActions.deflate, 1.5, deflate=['ld', 'frp', 'brp'])
    #     run_action(pub, WayneActions.deflate_all, 2.0)
    #     reset(pub)

    # if floaty:
    #     run_action(pub, WayneActions.deflate_all, 2.0)

    #     run_action(pub, WayneActions.inflate, 0.05, inflate=['ld', 'rd'])

    #     i = 0
    #     while (i < 5):
    #         # maybe reduce deflates in the morning
    #         run_action(pub, WayneActions.inflate, 0.15, inflate=['blp', 'brp', 'flp', 'frp', 'mp'])
    #         run_action(pub, WayneActions.deflate_all_but, 1.2, all_but=['ld', 'rd'])
    #         i += 1

    #     run_action(pub, WayneActions.deflate_all, 2.0)
    #     reset(pub)
 


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass