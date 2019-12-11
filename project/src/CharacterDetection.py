#!/usr/bin/env python

'''
Feature homography
==================

Example of using features2d templatework for video homography matching.
ORB features and FLANN matcher are used. The actual tracking is implemented by
PlaneTracker class in plane_tracker.py

Inspired by http://www.youtube.com/watch?v=-ZNYoL8rzPY

video: http://www.youtube.com/watch?v=FirtmYcC0Vc

Usage
-----
feature_homography_from_file1.py [<video source>]

'''

# Python 2/3 compatibility


# local modules
from __future__ import division
import common
from common import getsize, draw_keypoints
from plane_tracker import PlaneTracker

import cv2
import numpy as np
import rospy
import sys
import os


class characterDetection:
    def __init__(self):
        self.template = None
        self.paused = False
        self.recognised = False
        self.tracker = PlaneTracker()
        self.character = ""

		#Add template images to tracker
        scarlet = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'cluedo_images/scarlet.png')))
        mustard = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'cluedo_images/mustard.png')))
        plum = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'cluedo_images/plum.png')))
        peacock = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', 'cluedo_images/peacock.png')))

        scarlet_rect = (0,0,scarlet.shape[1],scarlet.shape[0])
        mustard_rect = (0,0,mustard.shape[1],mustard.shape[0])
        plum_rect = (0,0,plum.shape[1],plum.shape[0])
        peacock_rect = (0,0,peacock.shape[1],peacock.shape[0])

        self.tracker.add_target(scarlet.copy(), scarlet_rect,'scarlet')
        self.tracker.add_target(mustard.copy(), mustard_rect,'mustard')
        self.tracker.add_target(plum.copy(), plum_rect,'plum')
        self.tracker.add_target(peacock.copy(), peacock_rect,'peacock')

    def checkPoster(self, cv_image):
        if not self.recognised:
            self.frame = cv_image.copy()
            w, h = getsize(self.frame)
            vis = np.zeros((h, w, 3), np.uint8)

            vis[:h,:w] = self.frame

            tracked = self.tracker.track(self.frame)
            print(len(tracked))
            if len(tracked) > 0:
                rospy.loginfo("3")
                for tracked_ob in tracked:
                    rospy.loginfo ('Found ' + tracked_ob.target.data)
                    self.character = tracked_ob.target.data

                    # Calculate Homography
                    h, status = cv2.findHomography(tracked_ob.p0, tracked_ob.p1)
                    self.recognised = True


def main(args):
    rospy.init_node("characterDetection", anonymous=True)
    cD = characterDetection()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == '__main__':
    main(sys.argv)
