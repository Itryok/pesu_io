# Contour / shape detection - left off at 1:30:26

import cv2

import sys

import rospy
import numpy as np
import pandas as    pd
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

from time import sleep
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, PoseStamped

# rospy.init_node('autohector', anonymous=False)
vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)



# rospy.init_node('autohector', anonymous=False)
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


# def move(x):
#     if x==1:
#         print("im in")
#         forward()
#     # sys.exit(0)

def getContours(img,imgContour) :

    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # image, retrieval method (here, for outermost contours), approximation

    for cnt in contours : # contours - array of contours detected in image

        area = cv2.contourArea(cnt) # finds area of selected contour
        # print(area)
        cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3) # image copy, selected contour, (-1 to draw all contours), color, thickness
        if area > 500 : # selects only contours without too much noise (contours with area > 500 units)
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 0),3)  # image copy, selected contour, (-1 to draw all contours), color, thickness
            perimeter = cv2.arcLength(cnt, True) # contour, is closed(?)
            # print(perimeter)
            approx = cv2.approxPolyDP(cnt, 0.02 * perimeter, True) # contour, resolution, is closed(?)
            # print(approx) # gives corner points for each contour (shape)
            print(len(approx)) # prints number of vertices
            objCor = len(approx) # number of corners (higher the number, more likely to be a circle)
            x, y, w, h = cv2.boundingRect(approx) # coordinates of each shape

            if objCor == 4 : # evaluating number of corners to determine shape
                aspRatio = w / float(h)
                if aspRatio > 0.95 and aspRatio < 1.05 : # if width = height within error margin, then square
                    objType = 'Square'
                else :
                    objType = 'Quadrilateral'
            elif objCor == 5 :
                objType = 'Pentagon'
            elif objCor > 5 : # circle if large number of corners are detected (large being greater than 5 here)
                objType = 'Conic'
            else :
                objType = 'None'
            
            global detected
            

            cv2.rectangle(imgContour, (x,y), (x+w, y+h), (0, 255, 0), 2) # bounding rectangle (green for each detected shape)
            cv2.putText(imgContour, objType, (x + (w//2) - 10 , y + (h//2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
            if (objType == 'Square' or objType == 'Quadrilateral' or objType == 'Pentagon' or objType == 'Conic') :
                print(objType)
                detected = 1
                forward()
                sleep(0.1)
                # hover()
                sleep(0.1)
                # sleep(0.1)
                # move(x)
                hover()

            elif detected == 0:
                ccw()
                sleep(0.1)

            # while(detected):
            #     forward()
            #     sleep(0.1)
            
            # hover()
            # sleep(0.1)
            # elif detected == 1:
            #     print("something")
            #     forward()
            #     sleep(0.1)

            # elif not detected:
            #     print("no box found")
            #     ccw()
            #     # sleep(4)
            #     hover()
            #     # sleep(2)
    
    # print("no cont so ccw")
    # ccw()
    # sleep(0.1)
    # hover()
    # sleep(2)
    if detected==0:
        ccw()
        sleep(0.01)


bridge = CvBridge()

def stackImages(scale,imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape [:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None,scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        ver = hor
    return ver


def imageCallback(ros_image):

    global bridge
    global out
    # up()
    # sleep(1)
    # hover()
    # sleep(1)
    # cw()
    # sleep(1)
    try:
        img = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        print("img type", type(img))
    except CvBridgeError as e:
            print(e)

    ##############################################################
    # from this point on, 'img' is the target of processing
    ##############################################################

    # image processing
    imgContour = img.copy()
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (7,7), 1)
    imgCanny = cv2.Canny(imgGray,50,50)
    getContours(imgCanny,imgContour)        
    imgStacked = stackImages(0.5, ([img,imgContour]))
    cv2.imshow("Image", imgStacked)

    #cv2.imshow('Original', img)
    #cv2.imshow('Gray', imgGray)
    #cv2.imshow('Blur', imgBlur)
    #cv2.imshow('Canny', imgCanny)
    #cv2.imshow('Copy with contours', imgContour)

    #cv2.waitKey(0)


    
    ##############################################################
    cv2.waitKey(1)


def main():
    global detected
    detected = 0
    rospy.init_node('UAVRecordRaw', anonymous=True)

    # hover()
    # up()
    # hover()
    # sleep(0.05)

    
    imageTopic = "/front_cam/camera/image"
    image_sub = rospy.Subscriber(imageTopic,Image, imageCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")

    #out.release() 
    #print(type(image_sub))
    cv2.destroyAllWindows()
    print("Shutdown complete.")

if __name__ == '__main__':
    main();
   

