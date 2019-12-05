#! /usr/bin/env python

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


class CluedoFinder:

    MIN_CONTOUR_AREA = 750

    def __init__(self):
        self.cv_bridge = CvBridge()
        self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        #self.centralised = False
        self.image_detected = False
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        self.desired_velocity = Twist()
        while True:
            #if(self.centralised and not self.image_detected):
                #self.desired_velocity.linear.x = 0.1
                #self.desired_velocity.angular.z = 0
            if(self.image_detected):
                self.desired_velocity.linear.x = 0
                self.desired_velocity.angular.z = 0
                break;
            if (not self.image_detected):
                self.desired_velocity.angular.z = 0.2

            self.pub.publish(self.desired_velocity)

    def callback(self, data):
        try:
            # Get the normal camera feed
            camera_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")

            # Process the image and get the contours
            pre_processed_img = self.pre_process(camera_image)
            contours, h = cv2.findContours(pre_processed_img, 1, 2)

            width = np.size(pre_processed_img, 1)
            centre = (width / 2)

            # Iterate over the contours and find any that have 4 sides
            for cnt in contours:

                if self.is_four_sided(cnt):
                    # Check if the contour area is big enough to be the cluedo character
                    area = cv2.contourArea(cnt)
                    if area > self.MIN_CONTOUR_AREA:
                        cv2.drawContours(camera_image, [cnt], 0, (0, 0, 255), -1)
                        self.image_detected = True
            cv2.namedWindow("Camera_Feed")
            cv2.imshow("Camera_Feed", camera_image)
            cv2.waitKey(1)
        except CvBridgeError as cv_err:
            rospy.loginfo(rospy.get_caller_id + " Error: " + str(cv_err))

    def pre_process(self, image):
        """ Pre process the image to grayscale and then threshold it """
        grayscale_img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(grayscale_img, 70, 255, 1)
        return thresh

    def is_four_sided(self, contour):
        """ Returns if a contour is a four sided shape or not """
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        if len(approx) == 4:
            return True
        return False

    def get_centre(self, contour):
        """ Returns the centre of a contour """
        m = cv2.moments(contour)
        cx = int(m['m10'] / m['m00'])
        cy = int(m['m01'] / m['m00'])
        return cx, cy


if __name__ == '__main__':
    cluedofinder = CluedoFinder()
    rospy.init_node("cluedoFinder", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Exiting")
    finally:
        cv2.destroyAllWindows()
