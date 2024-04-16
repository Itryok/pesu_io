import cv2
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge, CvBridgeError

def getContours(imgContour) :

    contours, hierarchy = cv2.findContours(imgContour, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # image, retrieval method (here, for outermost contours), approximation

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

            cv2.rectangle(imgContour, (x,y), (x+w, y+h), (0, 255, 0), 2) # bounding rectangle (green for each detected shape)
            cv2.putText(imgContour, objType, (x + (w//2) - 10 , y + (h//2) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)



bridge = CvBridge()
"""
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

"""
def imageCallback(ros_image):

    global bridge
    global out

    try:
        img = bridge.imgmsg_to_cv2(ros_image, "bgr8")
        print("img type", type(img))
    except CvBridgeError as e:
            print(e)

    ##############################################################
    # from this point on, 'img' is the target of processing
    ##############################################################

    # image processing   
    #path = '/home/secondary/Desktop/pesuio/img.jpg'
    #img = cv2.imread(path)
    print(type(img))
    imgContour = img.copy()
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgBlur = cv2.GaussianBlur(imgGray, (7,7), 1)
    imgCanny = cv2.Canny(imgBlur, 50, 50)
    getContours(imgCanny)
    #cv2.imshow('Original', img)
    #cv2.imshow('Gray', imgGray)
    #cv2.imshow('Blur', imgBlur)
    #cv2.imshow('Canny', imgCanny)
    cv2.imshow('Copy with contours', imgContour)
    cv2.waitKey(0)
    #imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)   
    #imgStacked = stackImages(0.5, ([img, imgHSV]))
    cv2.imshow("Image", img)
    ##############################################################
    cv2.waitKey(1)


def main(args):

    rospy.init_node('UAVRecordRaw', anonymous=True)

    

    #out.release() 
    imageTopic = "/front_cam/camera/image"
    image_sub = rospy.Subscriber(imageTopic,Image, imageCallback)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")
    #print(type(image_sub))
    cv2.destroyAllWindows()
    print("Shutdown complete.")

if __name__ == "__main__":
    main(sys.argv)

