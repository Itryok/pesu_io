import sys
import math
import time
import csv

import rospy
import numpy as np
import pandas as    pd
import matplotlib.pyplot as plt
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped,Twist

from time import sleep
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry

global x,y,z,yaw
log_x = []
log_y = []
log_z = []
log_yaw = []


#Publishers
takeoff_pub = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=1)
land_pub = rospy.Publisher('/ardrone/land', Empty, queue_size=1)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    

def quaterionToRads(data):
    x = data.pose.orientation.x
    y = data.pose.orientation.y
    z = data.pose.orientation.z
    w = data.pose.orientation.w

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    yawZActual = math.atan2(t3, t4)
    if yawZActual < 0:
	    yawZActual = 2*math.pi + yawZActual

    return yawZActual

def pose_callback(data):
    global x,y,z

    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    z = data.pose.pose.position.z
    
    log_x.append(x)
    log_y.append(y)
    log_z.append(z)  
    



def rot_callback(data):
    global x,y,z,yaw 
    
    yaw = math.degrees(quaterionToRads(data))
    log_yaw.append(yaw)


def land_fun():
    land_pub.publish(Empty())

def enviar_velocidad(vx,vy,vz,vaz):
    vel_msg = Twist()
    vel_msg.linear.x = float(vx)
    vel_msg.linear.y = float(vy)
    vel_msg.linear.z = float(vz)
    vel_msg.angular.z = float(vaz)
    vel_msg.angular.x = float(0.0)
    vel_msg.angular.y = float(0.0)
    vel_pub.publish(vel_msg)

def hover_pub():
    enviar_velocidad(0.0,0.0,0.0,0.0)
    

def takeoff_fun():
    print("TakeOff")
    takeoff_pub.publish(Empty())

def up_fun():
    vel_msg = Twist()
    vel_msg.linear.z = float(1.0)
    vel_pub.publish(vel_msg)

def down_fun():
    vel_msg = Twist()
    vel_msg.linear.z = float(-1.0)
    vel_pub.publish(vel_msg)

def forward_fun():
    vel_msg = Twist()
    vel_msg.linear.x = float(1.0)
    vel_pub.publish(vel_msg)

def backward_fun():
    vel_msg = Twist()
    vel_msg.linear.x = float(-1.0)
    vel_pub.publish(vel_msg)

def right_fun():
    vel_msg = Twist()
    vel_msg.linear.y = float(-1.0)
    vel_pub.publish(vel_msg)

def left_fun():
    vel_msg = Twist()
    vel_msg.linear.y = float(1.0)
    vel_pub.publish(vel_msg)

def cw_fun():
    vel_msg = Twist()
    vel_msg.angular.z = float(-1.0)
    vel_pub.publish(vel_msg)

def ccw_fun():
    vel_msg = Twist()
    vel_msg.angular.z = float(1.0)
    vel_pub.publish(vel_msg)


def move(velocity_publisher,distance,speed,is_forward):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()
    #get current location 
    global x, y
    x0=x
    y0=y
    if (is_forward):
        velocity_message.linear.x =abs(speed)
    else:
        velocity_message.linear.x =-abs(speed)
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)           
    while True :
        rospy.loginfo("Drone moves forward")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()               
        distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print  (distance_moved)
        print(x)
        if  not (distance_moved<distance):
            rospy.loginfo("reached")
            break      
    #finally, stop the robot when the distance is moved
    velocity_message.linear.x =0
    velocity_publisher.publish(velocity_message)   

def move2(velocity_publisher,distance,speed,is_forward):
    #declare a Twist message to send velocity commands
    velocity_message = Twist()
    #get current location 
    global x, y
    x0=x
    y0=y
    if (is_forward):
        velocity_message.linear.x =abs(speed)
    else:
        velocity_message.linear.x =-abs(speed)
    distance_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)           
    while True :
        rospy.loginfo("Drone moves forward")
        velocity_publisher.publish(velocity_message)
        loop_rate.sleep()               
        distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
        print  (distance_moved)
        print(x)
        if  not (distance_moved<distance):
            rospy.loginfo("reached")
            break      
    #finally, stop the robot when the distance is moved
    velocity_message.linear.x =0
    velocity_publisher.publish(velocity_message)       
def rotate (velocity_publisher, angular_speed_degree, relative_angle_degree, clockwise):
    velocity_message = Twist()
    angular_speed=math.radians(abs(angular_speed_degree))
    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)
    angle_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    t0 = rospy.Time.now().to_sec()
    while True :
        rospy.loginfo("Drone rotates")
        velocity_publisher.publish(velocity_message)
        t1 = rospy.Time.now().to_sec()
        current_angle_degree = (t1-t0)*angular_speed_degree
        loop_rate.sleep()                    
        if  (current_angle_degree>relative_angle_degree):
            rospy.loginfo("reached")
            break
    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)


def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x
    global y, yaw
    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))
        linear_speed = distance * K_linear
        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular
        #velocity_message.linear.x = linear_speed
        #velocity_message.angular.z = angular_speed
        #velocity_publisher.publish(velocity_message)
        print ('x=', x, ', y=',y, ', distance to goal: ', distance)
        move(vel_pub,distance,linear_speed,True)
        if (distance <0.01):
            rospy.loginfo("reached")
            break
'''#finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)'''

def setDesiredOrientation(publisher, desired_angle_degree):
    global yaw
    while(yaw!=desired_angle_degree):
        cw_fun()


def move_in_preplanned_path(vel_pub):
    
    hover_pub()
    sleep(2)
    up_fun()
    sleep(3)
    hover_pub()
    sleep(2)
    rotate(vel_pub,15,0,True)
    move(vel_pub,5,5,True)
    rotate(vel_pub,15,90,True)
    move(vel_pub,5,5,True)
    rotate(vel_pub,15,90,True)
    move(vel_pub,5,5,True)
    rotate(vel_pub,15,90,True)
    move(vel_pub,5,5,True)
    down_fun()
    
def move_in_farmland_path(vel_pub):
    global y
    hover_pub()
    sleep(2)
    up_fun()
    sleep(1)
    hover_pub()
    sleep(2)
    rotate(vel_pub,10,0,True)
    go_to_goal(vel_pub,5,0)
    rotate(vel_pub,10,90,True)
    go_to_goal(vel_pub,5,-5)
    rotate(vel_pub,10,90,True)
    go_to_goal(vel_pub,0,-5)
    rotate(vel_pub,10,90,True)
    go_to_goal(vel_pub,0,0)
    down_fun()

def main(args):
    rospy.init_node('HectorQ_GUI', anonymous=False)
    #Subscribers
    posicionLider_sub = rospy.Subscriber("/ground_truth/state", Odometry , pose_callback)
    orientaLider_sub = rospy.Subscriber("/ground_truth_to_tf/pose", PoseStamped , rot_callback)
    move_in_preplanned_path(vel_pub)
    #move_in_farmland_path(vel_pub)
    keys = ['x_coord','y_coord','z_coord','yaw']
    values = [log_x,log_y,log_z,log_yaw]
    dict = {k:v for k,v in zip(keys,values)}
    with open('Logging.csv','w') as f:
        csv_writer = csv.writer(f)
        for data in dict.items():
          csv_writer.writerow(data)	
    with open('log.csv','w') as f:
        csv_writ = csv.writer(f)
        csv_writ.writerow(keys)
        for x,y,z,yw in zip(log_x,log_y,log_z,log_yaw):
           l=[[x,y,z,yw]]
           csv_writ.writerows(l)
    '''df = pd.DataFrame({'X_coor':log_x,'Y_coor':log_y,'Z_coor':log_z,'Yaw':log_yaw})
    df.to_csv('Logging.csv')'''
    plt.plot(log_x,log_y,'b.')
    plt.show()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")
    

if __name__ == "__main__":
    main(sys.argv)
