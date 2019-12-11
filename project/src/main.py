#! /usr/bin/env python

import rospy
import numpy as np
from room import RoomOne, RoomTwo
from go_to_specific_point_on_map import GoToPose
from CircleDetection import colourIdentifier
from cluedofinder import CluedoFinder
from CharacterDetection import characterDetection
import cv2


def moveBasedOnRedOrGreen(green, red, navigator):
    success = False
    if green:

        # Customize the following values so they are appropriate for your location
        x = room_1.get_centre("x")
        y = room_1.get_centre("y")
        theta = 0  # This is for rotation
        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta/2.0), 'r4': np.cos(theta/2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)
        if success:
            rospy.loginfo("Hooray, reached the desired pose")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

    elif red:
        rospy.init_node('move_to_point', anonymous=True)
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        x = room_2.get_entrance("x")
        y = room_2.get_entrance("y")
        theta = 0  # This is for rotation
        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta/2.0), 'r4': np.cos(theta/2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

    return success


if __name__ == '__main__':

    import sys

    if len(sys.argv) < 2:
        print("<Usage> python main.py path_to_input_points.yaml")
        exit(1)

    INPUT_POINTS_PATH = sys.argv[1]
    room_1 = RoomOne(INPUT_POINTS_PATH)
    room_2 = RoomTwo(INPUT_POINTS_PATH)
    green = False
    red = False

    success = False
    successGreen = False
    successMove = False

    try:
        rospy.init_node('move_to_point', anonymous=True)
        navigator = GoToPose()

        # Customize the following values so they are appropriate for your location
        x = room_1.get_entrance("x")
        y = room_1.get_entrance("y")
        theta = 0  # This is for rotation
        position = {'x': x, 'y': y}
        quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta/2.0), 'r4': np.cos(theta/2.0)}

        rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
        success = navigator.goto(position, quaternion)

        if success:
            rospy.loginfo("Hooray, reached the desired pose")
            try:
                while True:
                    cI=colourIdentifier()
                    if cI.green_circle_detected:
                        rospy.loginfo("GREEN")
                        green = True
                        cI.detect = False
                        break
                    elif cI.red_circle_detected:
                        rospy.loginfo("RED")
                        red = True
                        cI.detect = False
                        break
            except KeyboardInterrupt:
                exit(-1)
            rospy.loginfo("DETECTED ONE")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

        successMove = moveBasedOnRedOrGreen(green, red, navigator)
        if(red and successMove):
            red = False
            green = False
            try:
                while True:
                    cI=colourIdentifier()
                    if cI.green_circle_detected:
                        rospy.loginfo("GREEN")
                        green = True
                        cI.detect = False
                        break
                    elif cI.red_circle_detected:
                        rospy.loginfo("RED")
                        red = True
                        cI.detect = False
                        break
            except KeyboardInterrupt:
                exit(-1)
            rospy.loginfo("DETECTED ONE")

            successGreen = moveBasedOnRedOrGreen(green,red, navigator)

        if(green and successMove):
            successGreen = True

        if(successGreen):
            try:
                while True:
                    cF = CluedoFinder()
                    if cF.image_close_enough:
                        rospy.loginfo("FOUND IMAGE")
                        break

                while True:
                    cd = characterDetection()
                    cd.checkPoster(cF.cv_image)
                    if cd.recognised:
                        cv2.imwrite('cluedo_character.png', cF.cv_image)
                        with open('cluedo_character.txt', "w") as textFile:
                            textFile.write(cd.character)
                        rospy.loginfo("DETECTED CLUEDO CHARACTER")
                        break
            except KeyboardInterrupt:
                exit(-1)

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
        exit(0)

    except KeyboardInterrupt:
        rospy.loginfo("Keyoard interrupt.")
        exit(0)
