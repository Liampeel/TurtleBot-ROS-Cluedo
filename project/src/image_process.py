#!/usr/bin/env python

"""
    Processing script.
    It offers image processing routines
    used by other classes.
"""

# ROS and OpenCV packages
from __future__ import division
import os
import cv2
import numpy as np

# Template matching for image boundary
def temp_matching(template, cv_image):
    """
        Carries on a template matching
        to find the boundary of the detected
        and recognised iamge.
        Arguments:
            param1: MAT OpenCV image (template)
            param2: MAT OpenCV image (image to check)
        Returns:
            MAT: OpenCV image (with boundary draw on top)
    """
    template = cv2.imread(os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..', '..', 'cluedo_images/%(template)s.png' % locals())))
    template = cv2.resize(template, None, fx = 0.3, fy = 0.3, interpolation = cv2.INTER_CUBIC)
    t, w, h = template.shape[::-1]
    print("Shape: ", w, h, t)

    method = 'cv2.TM_CCOEFF_NORMED'

    # Apply template Matching
    res = cv2.matchTemplate(cv_image,
                            template,
                            eval(method))
    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

    # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
    if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        top_left = min_loc
    else:
        top_left = max_loc

    # Draw boundary on the image
    bottom_right = (top_left[0] + w, top_left[1] + h)
    cv2.rectangle(cv_image, top_left, bottom_right, 255, 2)
    return cv_image
