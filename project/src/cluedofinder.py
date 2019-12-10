#! /usr/bin/env python

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CluedoFinder:

    MIN_CONTOUR_AREA = 750
    CLOSE_ENOUGH_AREA = 30000

    def __init__(self):
        self.cv_bridge = CvBridge()
        self.subscriber = rospy.Subscriber('camera/rgb/image_raw', Image, self.callback)
        self.centralised = False
        self.image_detected = False
        self.image_close_enough = False
        self.pub = rospy.Publisher('mobile_base/commands/velocity', Twist)
        self.desired_velocity = Twist()
        self.cv_image = None
        self.anticlockwise = False


    def callback(self, data):
        try:
            # Get the normal camera feed
            camera_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
            if camera_image is not None:
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
                        if area > self.CLOSE_ENOUGH_AREA:
                            self.image_close_enough = True
                            self.cv_image = camera_image
                        elif area > self.MIN_CONTOUR_AREA:
                            self.image_detected = True
            
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
