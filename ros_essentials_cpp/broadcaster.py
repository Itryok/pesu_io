#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def run():
    channel1 = rospy.Publisher("topic1", String , queue_size=10)
    channel2 = rospy.Publisher("topic2", String , queue_size=10)
    rospy.init_node('Broadcaster',anonymous=True)
    rate = rospy.Rate(1)
    i=0
    while not rospy.is_shutdown():
        channel_str1 = "Channel_1 , counter =%s" % i
        rospy.loginfo(channel_str1)
        channel1.publish(channel_str1)
        channel_str2 = "Channel_2 , counter =%s" % i
        rospy.loginfo(channel_str2)
        channel2.publish(channel_str2)
        rate.sleep()
        i=i+1

if __name__ == '__main__':
    run()


