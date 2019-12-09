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
        self.detect = True
        while True:
            if(self.centralised and not (self.green_circle_detected or self.red_circle_detected)):
                self.desired_velocity.linear.x = 0.1
                self.desired_velocity.angular.z = 0
            if(self.green_circle_detected or self.red_circle_detected):
                self.desired_velocity.linear.x = 0
                self.desired_velocity.angular.z = 0
                break;
            if (not (self.green_detected or self.red_detected) and not (self.green_circle_detected or self.red_circle_detected or self.centralised)):
                self.desired_velocity.angular.z = 0.2

            self.pub.publish(self.desired_velocity)


    #Return a masked image based on lower and upper colour bounds
    def getColourMask(self,hsv, hsv_lower,hsv_upper):
        mask = cv2.inRange(hsv,hsv_lower,hsv_upper)
        return mask

    #Return the combination of two masks using bitwise_or
    def getCombinedMasks(self, mask1, mask2):
        combinedMask = cv2.bitwise_or(mask1,mask2)
        return combinedMask

    #return the image of the combined mask to display
    def getCominedMaskCamFeed(self, image, combinedMask):
        camFeed = cv2.bitwise_and(image, image, mask = combinedMask)
        return camFeed

    #return contours of a mask
    def getContours(self, mask):
        contours , hierarchy = cv2.findContours(mask.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        return contours, hierarchy

    #return converted colour of an image
    def convertColour(self, image, conversion):
        newImage = cv2.cvtColor(image, conversion)
        return newImage

    #return the maximum contours
    def getMaxContours(self, contours):
        maxContours = max(contours, key = cv2.contourArea)
        return maxContours

    #detect if the area of the maximum contour is greater than 1000,
    #if it is, we say that we have detected an object of that colour
    #return a boolean that states if the colour has been DETECTED
    #return the maximum contour
    def detectColour(self,contours):
        colourDetected = False;
        maxContour = self.getMaxContours(contours)
        if cv2.contourArea(maxContour) > 1000:
            colourDetected = True
        else:
            colourDetected = False
        return colourDetected, maxContour

    #get the centre points of the coloured object based on the moments of
    #the maximum contour
    def getCentrePoints(self, maxContour):
        M = cv2.moments(maxContour)
        cx = 0
        cy = 0
        try:
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
        except ZeroDivisionError:
            pass
        return cx, cy

    #draw a circle over the outline of the found circle
    def drawCircles(self,circles, cv_image):
        COLOUR_GREEN = np.array([0,255,0])
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            centre = (i[0], i[1])
            cv2.circle(cv_image, centre, 1, COLOUR_GREEN,3)
            radius = i[2]
        cv2.circle(cv_image,centre,radius,COLOUR_GREEN,3)

    #check that we are central with the found object
    def centralise(self, cx, cy, centre):
        centralised = False
        if((cx < centre + 30) and (cx > centre -30 )):
            centralised = True

    #detect if there is a circle in the image
    def detectCircle(self, maxContour, circles, cv_image):
        circle_detected = False
        if cv2.contourArea(maxContour) > 8000 and circles is not None:
            circle_detected = True
            self.drawCircles(circles,cv_image)
        return circle_detected

    #run the circle detection code
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            if not self.detect:
                return
        except CvBridgeError as e:
            print(e)
        except AttributeError:
            pass

        #Declare upper and lower bounds for red and green hsv values
        hsv_green_lower = np.array([40,10,10])
        hsv_green_upper = np.array([80,255,255])
        hsv_red_lower = np.array([0,50,50])
        hsv_red_upper = np.array([5,255,255])

        #Set the hsv image
        hsv = self.convertColour(cv_image, cv2.COLOR_BGR2HSV)

        #image mask declarations
        image_mask_green = self.getColourMask(hsv, hsv_green_lower, hsv_green_upper)
        image_mask_red = self.getColourMask(hsv, hsv_red_lower, hsv_red_upper)

        #Combined red and green mask
        red_green_mask = self.getCombinedMasks(image_mask_red, image_mask_green)

        #The combined masked feed to display at runtime
        image_res = self.getCominedMaskCamFeed(cv_image, red_green_mask)

        #Find contours in the masks
        contours_green, hierarchy = self.getContours(image_mask_green)
        contours_red, hierarchy = self.getContours(image_mask_red)

        #initialise greyscale image and blur it
        grey = self.convertColour(cv_image, cv2.COLOR_BGR2GRAY)
        grey = cv2.medianBlur(grey,9)

        #Use HoughCircles function to detect circles within the masked images.
        circles = cv2.HoughCircles(grey, cv.CV_HOUGH_GRADIENT,1,20,param1 = 50, param2= 30,minRadius = 90,maxRadius = 110)

        #finding the centre point of the combined mask
        width = np.size(image_res, 1)
        centre = (width / 2)

        #if we find any green contours
        if len(contours_green) > 0:
            #call the detect colour function
            self.green_detected, maxGreenContour = self.detectColour(contours_green)
            #find centre points of the green object
            cxGreen, cyGreen = self.getCentrePoints(maxGreenContour)
            #check if we are central to the green object
            self.centralised = self.centralise(cxGreen, cyGreen, centre)
            if self.green_detected == True:
                #if we have detected green, call the detect circle function to see if this is a green circle
                self.green_circle_detected = self.detectCircle(maxGreenContour, circles, cv_image)

        #if we find any red contours
        if len(contours_red) > 0:
            #call the detecxt colour function
            self.red_detected, maxRedContour = self.detectColour(contours_red)
            #find the centre points of the found red object
            cxRed, cyRed = self.getCentrePoints(maxRedContour)
            #check if we are central to the red object
            self.centralised = self.centralise(cxRed, cyRed, centre)
            if self.red_detected == True:
                #if we have detected red, call the detect circle function to see if this object is a red circle
                self.red_circle_detected = self.detectCircle(maxRedContour, circles, cv_image)

        # # Display feeds
        # cv2.namedWindow('Camera_Feed')
        # cv2.imshow("Camera_Feed", cv_image)
        # cv2.imshow('res',image_res)
        # cv2.waitKey(1)


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
