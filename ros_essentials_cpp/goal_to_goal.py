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



def go_to_goal(x_goal, y_goal,data):
    
    velocity_message = Twist()
    # cmd_vel_topic='/turtle1/cmd_vel'

    while (True):
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y

        print("x and y",x,y)

        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yawZActual)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        vel_pub.publish(velocity_message)
        # print 'x=', x, 'y=',y

        print("distance is",distance)
        if (distance <0.01):
            hover()

def yaw_callback(data):
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    z = data.pose.orientation.z
    w = data.pose.orientation.w

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    global yawZActual
    yawZActual = math.atan2(t3, t4)

    if yawZActual < 0:
        yawZActual = 2*math.pi + yawZActual
    
    # return yawZActual
	

def pose_callback(data):
    velocity_message = Twist()
    # cmd_vel_topic='/turtle1/cmd_vel'
    x_goal = 2
    y_goal = 2
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y

    yaw_sub = rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped , yaw_callback)


    print("x and y",x,y)
    print("yaw: ",yawZActual)

    # K_linear = 0.5 
    distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

    # linear_speed = distance * K_linear

    # K_angular = 4.0
    desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
    # angular_speed = (desired_angle_goal-yawZActual)*K_angular

    # velocity_message.linear.x = linear_speed
    # velocity_message.angular.z = angular_speed

    if abs(desired_angle_goal-yawZActual) > 0.1:
        velocity_message.linear.x = 0.0
        velocity_message.angular.z = 0.3
    else:
        velocity_message.linear.x = 0.5
        velocity_message.angular.z = 0.0

    vel_pub.publish(velocity_message)
        # print 'x=', x, 'y=',y

    print("distance is",distance)
    # if (distance <0.01):
    #     exit()
        # hover()
    # else:
    #     forward()
    





def main():

    rospy.init_node('go_to_goal', anonymous=True)

    positionTopic = "/ground_truth/state"

    hover()
    sleep(2)
    up()
    sleep(3)
    hover()
    sleep(2)
    posicionLider_sub = rospy.Subscriber(positionTopic, Odometry , pose_callback)

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
   
