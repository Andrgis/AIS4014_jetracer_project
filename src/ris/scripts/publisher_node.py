#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def talk_to_me():
    pub = rospy.Publisher('chatter_topic', String, queue_size=10)
    rospy.init_node('publisher_node', anonymous=True)
    rate = rospy.Rate(1)
    rospy.loginfo("Publisher node initialized.")
    while not rospy.is_shutdown():
        msg = "Hello, Andreas - %s" % rospy.get_time()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talk_to_me()
    except rospy.ROSInterruptException:
        pass