# using ground_truth rostopic moving the drone in a planned path 

import cv2
import sys
import time
import rospy
import numpy as np
import pandas as    pd
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError


import math
import rospy
from time import sleep
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

# rospy.init_node('hector_control', anonymous=False)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)


def initMessage(vx,vy,vz,vaz):
    vel_msg = Twist()
    vel_msg.linear.x = float(vx)
    vel_msg.linear.y = float(vy)
    vel_msg.linear.z = float(vz)
    vel_msg.angular.z = float(vaz)
    vel_msg.angular.x = float(0.0)
    vel_msg.angular.y = float(0.0)
    vel_pub.publish(vel_msg)

def hover():
    initMessage(0.0,0.0,0.0,0.0)

def up():
    vel_msg = Twist()
    vel_msg.linear.z = float(1.0)
    vel_pub.publish(vel_msg)

def down():
    vel_msg = Twist()
    vel_msg.linear.z = float(-1.0)
    vel_pub.publish(vel_msg)

def forward():
    vel_msg = Twist()
    vel_msg.linear.x = float(1.0)
    vel_pub.publish(vel_msg)

def backward():
    vel_msg = Twist()
    vel_msg.linear.x = float(-1.0)
    vel_pub.publish(vel_msg)

def right():
    vel_msg = Twist()
    vel_msg.linear.y = float(-1.0)
    vel_pub.publish(vel_msg)

def left():
    vel_msg = Twist()
    vel_msg.linear.y = float(1.0)
    vel_pub.publish(vel_msg)

def cw():
    vel_msg = Twist()
    vel_msg.angular.z = float(-1.0)
    vel_pub.publish(vel_msg)

def ccw():
    vel_msg = Twist()
    vel_msg.angular.z = float(1.0)
    vel_pub.publish(vel_msg)




def pose_callback(data):
    # x = data.pose.pose.position.x
    # y = data.pose.pose.position.y
    # z = data.pose.pose.position.z
    # while data.pose.pose.position.x>0:
    #     backward()
    #     sleep(0.1)
    #     print(data.pose.pose.position.x)
    #     # x = data.pose.pose.position.x
    # hover()
    # sleep(0.1)
    print(data.pose.pose.position.x)

    # if data.pose.pose.position.x<5:
    #     forward()
    forward()

    # if data.pose.pose.position.x>5 and data.pose.pose.position.y<5:
    #     print("hello")
    #     left()
    # if data.pose.pose.position.x>0 and data.pose.pose.position.y>5:
    #     print("hello_2")
    #     backward()
    # if data.pose.pose.position.x>0 and data.pose.pose.position.y>5:
    #     print("hello_3")
    #     right()

    while(data.pose.pose.position.x>5):
        print(data.pose.pose.position.y)
        left()
    
    # sleep(3)
    # hover()
    # sleep(2)
    # return 0



bridge = CvBridge()

def imageCallback(ros_image):

    global bridge
    global out

    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        # print("img type", type(frame))
    except CvBridgeError as e:
            print(e)
    imageTopic = "/front_cam/camera/image"
    cap = cv2.VideoCapture(imageTopic)
    c1 = 0
    linecolor = (100, 215, 255)
    lwr_red = np.array([0, 0, 0])
    upper_red = np.array([179, 65, 55])
    width = cap.get(3)

    frame = frame[:, 0:320]
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.inRange(hsv, lwr_red, upper_red)
    mask = cv2.dilate(mask, kernel, iterations=1)
    res = cv2.bitwise_and(frame, frame, mask=mask)
    cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    center = None
    
    
    # cv2.imshow("hsv",hsv)
    cv2.imshow("Frame", frame)
    # cv2.imshow("contour",mask)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()

def main():

    rospy.init_node('UAVRecordRaw', anonymous=True)

    
    imageTopic = "/front_cam/camera/image"
    positionTopic = "/ground_truth/state"

    # vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    hover()
    sleep(2)
    up()
    sleep(3)
    hover()
    sleep(2)
    posicionLider_sub = rospy.Subscriber(positionTopic, Odometry , pose_callback)

    # image_sub = rospy.Subscriber(imageTopic,Image, imageCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")

    #out.release() 
    #print(type(image_sub))
    cv2.destroyAllWindows()
    print("Shutdown complete.")

if __name__ == '__main__':
    main()
