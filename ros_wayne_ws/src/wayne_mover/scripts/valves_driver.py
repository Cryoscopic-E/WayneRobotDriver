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

# config with rear legs together
# VALVES_PINS = {
#     "inflate":0,
#     "deflate":1,
#     "rl": 2,
#     "mll":4,
#     "fll":6,
#     "frl":8,
#     "mrl":10,
#     "rb":12,
#     "lb":14
# }

# config with middle legs together
VALVES_PINS = {
    "inflate":0,
    "deflate":1,
    "ml": 2,
    "rll":4,
    "fll":6,
    "frl":8,
    "rrl":10,
    "rb":12,
    "lb":14
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
            print(' '*12 + f'Inflate: {a}')
        elif action == WayneActions.deflate:
            a = deflate(**kwargs)
            print(' '*12 + f'Deflate: {a}')
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
            print(f'Inflate and Deflate: {a}')
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
    
    # run_action(pub, WayneActions.inflate, 0.3, inflate=['rb', 'frl'])
    # run_action(pub, WayneActions.deflate, 2.0, deflate=['rb', 'frl'])
    # run_action(pub, WayneActions.hold, 5.0)
    # run_action(pub, WayneActions.inflate_all, 0.3)
    # run_action(pub, WayneActions.deflate_all, 2.0)
    # run_action(pub, WayneActions.inflate_all_but, 0.3, all_but=['rb', 'frl'])
    # run_action(pub, WayneActions.deflate_all_but, 2.0, all_but=['rb', 'frl'])
    # run_action(pub, WayneActions.inflate_xs_deflate_ys, 0.4, inflate=['rb', 'frl'], deflate=['lb', 'rll'])
    # run_action(pub, WayneActions.deflate_all, 2.0)

    run_action(pub, WayneActions.deflate_all, 3.0)

    reset(pub)

# def main():
#     # add actions and sequences folder
#     script_folder = os.path.dirname(os.path.abspath(__file__))
#     action_file = os.path.join(script_folder, 'sequences/actions_v2.yml')
#     sequence_file = os.path.join(script_folder, 'sequences/sequence_test_walk2.yml')
   

#     # load actions
#     with open(action_file) as file:
#         actions = yaml.safe_load(file)

#     # load sequences
#     with open(sequence_file) as file:
#         sequence = yaml.safe_load(file)

#     rospy.init_node('test_wayne_board', anonymous=True)
#     pub = rospy.Publisher('register_state', UInt8MultiArray, queue_size=10)
#     rate = rospy.Rate(1)


#     while not pub.get_num_connections() > 0:
#         rospy.loginfo('Waiting for RegisterState subscriber')
#         rospy.sleep(1)
    
#     rospy.loginfo('Starting sequence..')
#     rospy.sleep(1)

#     # --------------
#     # --- NEEDED ---
#     # --------------
#     # SET HOW MANY ITERATIONS YOU QANT TO RUN #
#     n_iter = 1
#     for i in range(n_iter):
#         for seq in sequence:
#             rospy.loginfo(f'Executing {seq["action"]} for {seq["seconds"]} seconds')
#             publish(pub, actions[seq['action']])
#             rospy.sleep(seq['seconds'])

  

#     # --------------
#     # --- NEEDED ---
#     # --------------
#     # SE SECONDS FOR DEFLATING ALL
#     deflate_all_sec = 5

#     # DEFLATE ALL AT THE END #
#     rospy.loginfo(f'Executing "Deflate All" for {deflate_all_sec} seconds')
#     publish(pub, actions['Deflate All'])
#     rospy.sleep(deflate_all_sec)


#     # RESETTING BEFORE CLOSING PROGRAM #
#     rospy.loginfo("Resetting..")
#     rate.sleep()
#     reset(pub)
#     rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass