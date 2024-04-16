#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def topic1_callback(message):
    rospy.loginfo(rospy.get_caller_id() + "Receiving %s", message.data)

def topic2_callback(message):
    rospy.loginfo(rospy.get_caller_id() + "Receiving %s", message.data)   
    
def receiver():
    rospy.init_node('Receiver', anonymous=True)
    rospy.Subscriber("topic1", String, topic1_callback)
    rospy.Subscriber("topic2", String, topic2_callback)
    rospy.spin()
    
if __name__ == '__main__':
    receiver()
