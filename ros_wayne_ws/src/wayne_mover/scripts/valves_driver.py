import rospy
from time import sleep
from std_msgs.msg import UInt8MultiArray
# from std_srvs.srv import Trigger

def publish(publisher : rospy.Publisher):
    msg = UInt8MultiArray()
    msg.data = [0,1,1,0,0,1,0,0,0,1,1,0,0,0,0,0]
    publisher.publish(msg)

def reset(publisher : rospy.Publisher):
    msg = UInt8MultiArray()
    msg.data = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
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
    rospy.init_node('test_wayne_board', anonymous=True)
    pub = rospy.Publisher('register_state', UInt8MultiArray, queue_size=10)
    rate = rospy.Rate(1)

    while not pub.get_num_connections() > 0:
        rospy.loginfo('Waiting for RegisterState subscriber')
        sleep(1)

    for i in range(5):
        publish(pub)
        rate.sleep()

    rospy.loginfo("Resetting..")
    reset(pub)
    # rospy.loginfo("Resetting..")
    # rospy.loginfo(reset_client())



if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass