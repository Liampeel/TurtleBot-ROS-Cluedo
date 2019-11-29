#!/usr/bin/env python

from __future__ import division
import cv2
import numpy as np
import rospy
import sys

from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String

class colourIdentifier():

    def __init__(self):
        pub = rospy.Publisher('chatter', String, queue_size=10)
        rate = rospy.Rate(10) # 10hz

    # Initialise a publisher to publish messages to the robot base
    # We covered which topic receives messages that move the robot in the 2nd Lab Session

        self.green = False

    	self.sensitivity = 10
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera/rgb/image_raw", Image, self.callback)

        #MAKE IT SPIN
        #IMPLEMENT RED DETECTION
        while not self.green and not self.red:
            if(self.green == True):
                pub.publish("green")
            	rate.sleep()
            else:
            	rate.sleep()


    def callback(self, data):
        try:
            image1 = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        hsvImage = cv2.cvtColor(image1, cv2.COLOR_BGR2HSV)

        hsv_green_lower = np.array([60 - self.sensitivity, 100, 100])
        hsv_green_upper = np.array([60 + self.sensitivity, 255, 255])


        maskG = cv2.inRange(hsvImage, hsv_green_lower, hsv_green_upper)




        contoursG = cv2.findContours(maskG, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]


        # Find the contours that appear within the certain colours mask using the cv2.findContours() method
        # For <mode> use cv2.RETR_LIST for <method> use cv2.CHAIN_APPROX_SIMPLE
        if len(contoursG) > 0 :

            contourMax = max(contoursG, key = cv2.contourArea)
            M = cv2.moments(contourMax)
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            # Loop over the contours
            # There are a few different methods for identifying which contour is the biggest
            # Loop throguht the list and keep track of whioch contour is biggest or
            # Use the max() method to find the largest contour
            # M = cv2.moments(c)
            # cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            #Check if the area of the shape you want is big enough to be considered
            # If it is then change the flag for that colour to be True(1)
            if cv2.contourArea > 50:

            # draw a circle on the contour you're identifying as a blue object as well
                cv2.circle(image1,(cx,cy),5,(255,0,0),1)
                self.green = True

            else:
                self.green = False
        else:
            self.green = False
            # Then alter the values of any flags
                    #Show the resultant images you have created. You can show all of them or just the end result if you wish to.
        cv2.namedWindow('Camera_Feed')
        cv2.imshow('Camera_Feed', image1)
        cv2.waitKey(3)

    #Show the resultant images you have created. You can show all of them or just the end result if you wish to.

# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main(args):
    rospy.init_node("image_converter", anonymous=True)
    cI = colourIdentifier()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
