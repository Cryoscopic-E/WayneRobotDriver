from time import sleep
import rospy
from std_msgs.msg import Empty

def usage():
    print('Use the following command:')
    print('rosrun wayne_mover test_board_pub.py x y')
    print('x: number of time to publish')
    print('y: delay in ms')

def publish(publisher : rospy.Publisher):
    publisher.publish(Empty())

def main():
    node = rospy.init_node('test_wayne_board', anonymous=True)
    pub = rospy.Publisher('toggle', Empty, queue_size=10)

    n_loops = rospy.get_param('n', default="int")
    n_delay = rospy.get_param('d', default="int")
    
    while not pub.get_num_connections() > 0:
        rospy.loginfo('Waiting for subscriber')
        sleep(1)

    rate = rospy.Rate(1000/n_delay)
    rospy.loginfo(f'RATE: {rate.sleep_dur}')
    
    

    for n in range(n_loops):
        publish(pub)
        rate.sleep()
    

if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass