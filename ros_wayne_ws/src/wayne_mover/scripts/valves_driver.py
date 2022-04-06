import rospy
from time import sleep
from std_msgs.msg import UInt8MultiArray
# from std_srvs.srv import Trigger

def publish(publisher : rospy.Publisher, data):
    msg = UInt8MultiArray()
    msg.data = data
    publisher.publish(msg)

def deflate(publisher : rospy.Publisher, rate: rospy.Rate):
    msg = UInt8MultiArray()
    msg.data = [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]
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
    rate.sleep()
    for i in range(2): # Inflating front
        publish(pub,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        rate.sleep()
    for i in range(2): # Inflate middle
        publish(pub,[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        rate.sleep()
    for i in range(10): # Deflate front
        publish(pub,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1])
        rate.sleep()
    for i in range(2): # Inflate Rear
        publish(pub,[0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0])
        rate.sleep()
    for i in range(10): # Deflating middle
        publish(pub,[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1])
        rate.sleep()
    for i in range(2): # Inflating front
        publish(pub,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        rate.sleep()
    for i in range(2): # Inflate middle
        publish(pub,[0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0])
        rate.sleep()
    for i in range(10):# Deflate front
        publish(pub,[1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1])
        rate.sleep()
    rospy.loginfo("Resetting..")
    deflate(pub)
    rate.sleep()
    reset(pub)
    rate.sleep()
    # rospy.loginfo("Resetting..")
    # rospy.loginfo(reset_client())



if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass