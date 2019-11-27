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
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)

        while not (self.green_detected or self.red_detected):
            self.desired_velocity.angular.z = 0.2
            self.pub.publish(self.desired_velocity)
            if(self.green_detected == True or self.red_detected == True):
            	self.desired_velocity.angular.z = 0


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)
        hsv_green_lower = np.array([40,10,10])
        hsv_green_upper = np.array([80,255,255])
        hsv_red_lower = np.array([0,50,50])
        hsv_red_upper = np.array([5,255,255])
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        image_mask_green = cv2.inRange(hsv,hsv_green_lower,hsv_green_upper)
        image_mask_red = cv2.inRange(hsv,hsv_red_lower,hsv_red_upper)
        red_green_mask = cv2.bitwise_or(image_mask_red,image_mask_green)
        image_res = cv2.bitwise_and(cv_image,cv_image, mask= red_green_mask)
        contours_green, hierarchy = cv2.findContours(image_mask_green.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, hierarchy = cv2.findContours(image_mask_red.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        green_circle = cv2.HoughCircles(image_mask_green, cv.CV_HOUGH_GRADIENT, 10, 300, param1 = 300, param2 = 200)
        red_circle = cv2.HoughCircles(image_mask_red, cv.CV_HOUGH_GRADIENT, 10, 300, param1 = 300, param2 = 200)

        width = np.size(image_res, 1)
        centre = (width / 2)
        if len(contours_green) > 0:
            cgreen = max(contours_green, key = cv2.contourArea)
            M = cv2.moments(cgreen)
            cxgreen, cygreen = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            # M = cv2.moments(c)
            # cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            # cxgreen, cygreen = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(cgreen) > 1000:

                self.green_detected = True
                # draw a circle on the contour you're identifying as a blue object as well
                COLOUR_GREEN = np.array([0,255,0])
                #COLOUR_RED = np.array([255,0,0])
                #cv2.circle(cv_image,(cx,cy), 50, COLOUR_BLUE, 1)
                cv2.circle(cv_image,(cxgreen,cygreen), 25, COLOUR_GREEN, 1)
                #COLOUR_BLUE = np.array([0,0,255])
                #COLOUR_GREEN = np.array([0,255,0])
                #COLOUR_RED = np.array([255,0,0])
                #cv2.circle(cv_image,(cx,cy), 50, COLOUR_BLUE, 1)
                #cv2.circle(cv_image,(cxgreen,cygreen), 25, COLOUR_GREEN, 1)
                print("Green detected")

                if green_circle is not None:
                    print("Circle detected")
                    self.green_circle_detected = True
                #cv2.circle(cv_image,(cx,cy), 50, COLOUR_RED, 1)
                # cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                # Then alter the values of any flags
        else:
            self.green_detected = False

        if len(contours_red) > 0:
            cred = max(contours_red, key = cv2.contourArea)
            M = cv2.moments(cred)

            # M = cv2.moments(c)
            # cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
            # cxgreen, cygreen = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea(cred) > 1000:

                self.red_detected = True
                # draw a circle on the contour you're identifying as a blue object as well
                #COLOUR_BLUE = np.array([0,0,255])
                #COLOUR_GREEN = np.array([0,255,0])
                #COLOUR_RED = np.array([255,0,0])
                #cv2.circle(cv_image,(cx,cy), 50, COLOUR_BLUE, 1)
                #cv2.circle(cv_image,(cxgreen,cygreen), 25, COLOUR_GREEN, 1)
                print("Red detected")

                if red_circle is not None:
                    print("Circle detected")
                    self.red_circle_detected = True
                #cv2.circle(cv_image,(cx,cy), 50, COLOUR_RED, 1)
                # cv2.circle(<image>,(<center x>,<center y>),<radius>,<colour (rgb tuple)>,<thickness (defaults to 1)>)
                # Then alter the values of any flags
        else:
            self.red_detected = False





        cv2.namedWindow('Camera_Feed')
        cv2.imshow("Camera_Feed", cv_image)
        cv2.imshow('res',image_res)
        cv2.waitKey(3)



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
