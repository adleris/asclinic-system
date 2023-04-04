#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    # Setup publisher (<topic_name>, <message_type>, Queue size)
    pub = rospy.Publisher('topic_Matt_chatter', String, queue_size=10)
    # Init node
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "Matt Pub: %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass