# Contour / shape detection - left off at 1:30:26

import cv2

import sys

import time

import rospy
import numpy as np
import pandas as    pd
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

def getContours(imgContour) :
    print("im inside getContours fun")

    contours, hierarchy = cv2.findContours(imgContour, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # image, retrieval method (here, for outermost contours), approximation

    for cnt in contours : # contours - array of contours detected in image
        print("im inside for loop")
        area = cv2.contourArea(cnt) # finds area of selected contour
        # print(area)
        cv2.drawContours(imgContour, cnt, -1, (255, 0, 0), 3) # image copy, selected contour, (-1 to draw all contours), color, thickness
        if area > 0 : # selects only contours without too much noise (contours with area > 500 units)
            print("valid contour")
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

            cv2.rectangle(imgContour, (x,y), (x+w, y+h), (0, 255, 0), 2) # bounding rectangle (green for each detected shape)
            cv2.putText(imgContour, objType, (x + (w//2) - 10 , y + (h//2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)



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

    try:
        frame = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        print("img type", type(frame))
    except CvBridgeError as e:
            print(e)

    ##############################################################
    # from this point on, 'frame' is the target of processing
    ##############################################################

    # image processing
    # imgContour = img.copy()
    # imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # imgBlur = cv2.GaussianBlur(imgGray, (7,7), 1)
    # imgCanny = cv2.Canny(imgGray,50,50)
    # getContours(imgCanny)        
    # imgStacked = stackImages(0.5, ([img,imgCanny]))
    # cv2.imshow("Image", imgStacked)

    # cv2.imshow("single_original image",img)

    #cv2.imshow('Original', img)
    #cv2.imshow('Gray', imgGray)
    #cv2.imshow('Blur', imgBlur)
    #cv2.imshow('Canny', imgCanny)
    #cv2.imshow('Copy with contours', imgContour)

    #cv2.waitKey(0)
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

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        if radius > 3:
            # cv2.circle(frame, (int(x), int(y)), int(radius), (255, 255, 255), 2)
            cv2.circle(frame, center, 5, linecolor, -1)

        if (x > 0 and x <= 0.25 * width):
            print("Left")
            cv2.putText(frame, '<--', (5, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
            time.sleep(0.01)

        elif (x > 0.25 * width and x <= 0.75 * width):
            print('Forward')
            cv2.putText(frame, '^', (5, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
            time.sleep(0.01)

        elif (x > 0.75 * width and x <= width):
            print("Right")
            cv2.putText(frame, '-->', (5, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
            time.sleep(0.01)
    else:
        print("Track Not Visible")
        c1 += 1
        if (c1 == 5):
            print("Backward")
            cv2.putText(frame, 'V', (5, 50), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2, cv2.LINE_AA)
            c1 = 0

    cv2.imshow("Frame", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cap.release()
        cv2.destroyAllWindows()


    
    ##############################################################
    # cv2.waitKey(1)


def main():

    rospy.init_node('UAVRecordRaw', anonymous=True)

    
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
    main()
   
