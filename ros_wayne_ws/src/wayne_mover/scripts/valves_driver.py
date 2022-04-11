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


# def deflate(publisher: rospy.Publisher, rate: rospy.Rate):
#     msg = UInt8MultiArray()
#     msg.data = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1]
#     publisher.publish(msg)


def reset(publisher: rospy.Publisher):
    msg = UInt8MultiArray()
    msg.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    publisher.publish(msg)

# def reset_client():
#     rospy.wait_for_service('reset_srv')
#     try:
#         service = rospy.ServiceProxy('reset_srv', Trigger)
#         resert_response = service()
#         return resert_response.success
#     except rospy.ServiceException as e:
#         rospy.loginfo("Service call failed: %s"%e)


def main():
    # add folder
    script_folder = os.path.dirname(os.path.abspath(__file__))
    action_file = os.path.join(script_folder, 'sequences/actions_v2.yml')
    sequence_file = os.path.join(script_folder, 'sequences/sequence_test_walk.yml')
    sequence_deflateall_file = os.path.join(script_folder, 'sequences/sequence_test_deflateall.yml')
    # load actions
    with open(action_file) as file:
        actions = yaml.safe_load(file)
    # load sequences
    with open(sequence_file) as file:
        sequence = yaml.safe_load(file)

    with open(sequence_deflateall_file) as file:
        sequence_deflate_all = yaml.safe_load(file)

    rospy.init_node('test_wayne_board', anonymous=True)
    pub = rospy.Publisher('register_state', UInt8MultiArray, queue_size=10)
    rate = rospy.Rate(1)


    while not pub.get_num_connections() > 0:
        rospy.loginfo('Waiting for RegisterState subscriber')
        rospy.sleep(1)
    
    rospy.loginfo('Starting sequence..')
    rospy.sleep(1)

    # for seq in sequence:
    #     rospy.loginfo(f'Executing {seq["action"]} for {seq["seconds"]} seconds')
    #     publish(pub, actions[seq['action']])
    #     rospy.sleep(seq['seconds'])
    
    # for seq in sequence_deflate_all:
    #     rospy.loginfo(f'Executing {seq["action"]} for {seq["seconds"]} seconds')
    #     publish(pub, actions[seq['action']])
    #     rospy.sleep(seq['seconds'])

    for i in range(50):
        for seq in sequence:
            rospy.loginfo(f'Executing {seq["action"]} for {seq["seconds"]} seconds')
            publish(pub, actions[seq['action']])
            rospy.sleep(seq['seconds'])

    for seq in sequence_deflate_all:
        rospy.loginfo(f'Executing {seq["action"]} for {seq["seconds"]} seconds')
        publish(pub, actions[seq['action']])
        rospy.sleep(seq['seconds'])

    # for i in range(seq['seconds']):
        
    #     rate.sleep()
    

    # for i in range(2): # Inflating front
    #     publish(pub,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    #     rate.sleep()
    # for i in range(2): # Inflate middle
    #     publish(pub,[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    #     rate.sleep()
    # for i in range(10): # Deflate front
    #     publish(pub,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1])
    #     rate.sleep()
    # for i in range(2): # Inflate Rear
    #     publish(pub,[0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0])
    #     rate.sleep()
    # for i in range(10): # Deflating middle
    #     publish(pub,[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1])
    #     rate.sleep()
    # for i in range(2): # Inflating front
    #     publish(pub,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    #     rate.sleep()
    # for i in range(2): # Inflate middle
    #     publish(pub,[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
    #     rate.sleep()
    # for i in range(10):# Deflate front
    #     publish(pub,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1])
    #     rate.sleep()

    
    
    
    # for i in range(20):
    #     deflate(pub, rate)
    #     rate.sleep()

    rospy.loginfo("Resetting..")
    rate.sleep()
    reset(pub)
    rate.sleep()

    # rospy.loginfo("Resetting..")
    # rospy.loginfo(reset_client())


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
