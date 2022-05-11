import rospy
import yaml
import os
from time import sleep
from std_msgs.msg import UInt8MultiArray
# from std_srvs.srv import Trigger


def publish(publisher: rospy.Publisher, data):
    msg = UInt8MultiArray()
    msg.data = data
    publisher.publish(msg)


def reset(publisher: rospy.Publisher):
    msg = UInt8MultiArray()
    msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    publisher.publish(msg)


def main():
    # add actions and sequences folder
    script_folder = os.path.dirname(os.path.abspath(__file__))
    action_file = os.path.join(script_folder, 'sequences/actions_v2.yml')
    sequence_file = os.path.join(script_folder, 'sequences/sequence_test_walk2.yml')
   

    # load actions
    with open(action_file) as file:
        actions = yaml.safe_load(file)

    # load sequences
    with open(sequence_file) as file:
        sequence = yaml.safe_load(file)

    rospy.init_node('test_wayne_board', anonymous=True)
    pub = rospy.Publisher('register_state', UInt8MultiArray, queue_size=10)
    rate = rospy.Rate(1)


    while not pub.get_num_connections() > 0:
        rospy.loginfo('Waiting for RegisterState subscriber')
        rospy.sleep(1)
    
    rospy.loginfo('Starting sequence..')
    rospy.sleep(1)

    # --------------
    # --- NEEDED ---
    # --------------
    # SET HOW MANY ITERATIONS YOU QANT TO RUN #
    n_iter = 1
    for i in range(n_iter):
        for seq in sequence:
            rospy.loginfo(f'Executing {seq["action"]} for {seq["seconds"]} seconds')
            publish(pub, actions[seq['action']])
            rospy.sleep(seq['seconds'])

  

    # --------------
    # --- NEEDED ---
    # --------------
    # SE SECONDS FOR DEFLATING ALL
    deflate_all_sec = 5

    # DEFLATE ALL AT THE END #
    rospy.loginfo(f'Executing "Deflate All" for {deflate_all_sec} seconds')
    publish(pub, actions['Deflate All'])
    rospy.sleep(deflate_all_sec)


    # RESETTING BEFORE CLOSING PROGRAM #
    rospy.loginfo("Resetting..")
    rate.sleep()
    reset(pub)
    rate.sleep()



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass