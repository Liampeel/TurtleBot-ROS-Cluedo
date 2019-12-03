#!/usr/bin/env python

from __future__ import division
import cv2
import cv2.cv as cv
import numpy as np
import rospy
import sys
import argparse
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
# import the necessary packages

class colourIdentifier():

    def __init__(self):

        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        self.desired_velocity = Twist()
        self.green_detected = False
        self.green_circle_detected = False
        self.red_detected = False
        self.red_circle_detected = False
        self.centralised = False
        self.bridge = CvBridge()
        self.stop = False
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")


        except CvBridgeError as e:
            print(e)
        cv_image = cv2.medianBlur(cv_image,5)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        #Declare upper and lower bounds for red and green hsv values
        hsv_green_lower = np.array([40,10,10])
        hsv_green_upper = np.array([80,255,255])
        hsv_red_lower = np.array([0,50,50])
        hsv_red_upper = np.array([5,255,255])


        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        #image mask declarations
        image_mask_green = cv2.inRange(hsv,hsv_green_lower,hsv_green_upper)
        image_mask_red = cv2.inRange(hsv,hsv_red_lower,hsv_red_upper)
        #image_mask_green = cv2.medianBlur(image_mask_green,3)
        #Combined red and green mask
        red_green_mask = cv2.bitwise_or(image_mask_red,image_mask_green)

        #The combined masked feed to display at runtime
        image_res = cv2.bitwise_and(cv_image,cv_image, mask= red_green_mask)

        #Find contours in the masks
        contours_green, hierarchy = cv2.findContours(image_mask_green.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, hierarchy = cv2.findContours(image_mask_red.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        grey = cv2.medianBlur(grey,9)
        #Use HoughCircles function to detect circles within the masked images.
        # green_circle = cv2.HoughCircles(image_mask_green, cv.CV_HOUGH_GRADIENT,1,20,param1 = 50, param2= 30,minRadius = 0,maxRadius = 0)#2, 32.0, 30, 550)
        # red_circle = cv2.HoughCircles(image_mask_red, cv.CV_HOUGH_GRADIENT, 100, 120, param1 = 150, param2 = 150)#, param1 = 500, param2 = 400, minRadius = 1, maxRadius = 200 )
        #print(str(green_circle) + "/n")
        circles = cv2.HoughCircles(grey, cv.CV_HOUGH_GRADIENT,1,20,param1 = 50, param2= 30,minRadius = 90,maxRadius = 110)
        width = np.size(image_res, 1)
        centre = (width / 2)


        #green circle detection
        if len(contours_green) > 0:
            cgreen = 0
            for cnts in contours_green:
                contourArea = cv2.contourArea(cnts)
                if contourArea > cgreen:
                    cgreen = contourArea
            M = cv2.moments(cgreen)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            #if cv2.contourArea(cgreen) > 1000:
            self.green_detected = True
            if((cx < centre + 10) and (cx > centre -10 )):
                self.centralised = True;

            if cgreen > 8000 and circles is not None:

                COLOUR_GREEN = np.array([0,255,0])
                circles = np.uint16(np.around(circles))
                for i in circles[0, :]:
                    centre = (i[0], i[1])
                    cv2.circle(cv_image, centre, 1, COLOUR_GREEN,3)
                    radius = i[2]
                    print i[2]
                    cv2.circle(cv_image,centre,radius,COLOUR_GREEN,3)
                self.green_circle_detected = True

        else:
            self.green_detected = False

        #Red circle detection
        if len(contours_red) > 0:
            cred = max(contours_red, key = cv2.contourArea)
            M = cv2.moments(cred)
            if cv2.contourArea(cred) > 1000:
                print(cv2.contourArea(cred))
                self.red_detected = True
                print("Red detected")
                if red_circle is not None and cv2.contourArea(cred) > 8000:
                    print("Circle detected")
                    COLOUR_GREEN = np.array([0,255,0])
                    red_circle = np.uint16(np.around(red_circle))
                    for i in red_circle[0, :]:
                        centre = (i[0], i[1])
                        cv2.circle(cv_image, centre, 1, COLOUR_GREEN,3)
                        radius = i[2]
                        cv2.circle(cv_image,centre,radius,COLOUR_GREEN,3)
                    self.red_circle_detected = True
        else:
            self.red_detected = False

        #Display feeds
        cv2.namedWindow('Camera_Feed')
        cv2.imshow("Camera_Feed", cv_image)
        cv2.imshow('res',image_res)
        cv2.waitKey(1)


def main(args):
    rospy.init_node("colourIdentifier", anonymous=True)
    cI = colourIdentifier()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)
