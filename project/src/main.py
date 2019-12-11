#! /usr/bin/env python

import rospy
import numpy as np
from room import RoomOne, RoomTwo
from go_to_specific_point_on_map import GoToPose
from CircleDetection import colourIdentifier
from cluedofinder import CluedoFinder
from CharacterDetection import characterDetection
import cv2

green = False
red = False
successMove = False
navigator = None
room_1 = None
room_2 = None


def moveBasedOnRedOrGreen(location, room):
    global navigator

    if location == "entrance":
        x = room.get_entrance("x")
        y = room.get_entrance("y")
    else:
        x = room.get_centre("x")
        y = room.get_centre("y")

    theta = 0  # This is for rotation
    position = {'x': x, 'y': y}
    quaternion = {'r1': 0.000, 'r2': 0.000, 'r3': np.sin(theta / 2.0), 'r4': np.cos(theta / 2.0)}

    rospy.loginfo("Go to (%s, %s) pose", position['x'], position['y'])
    return navigator.goto(position, quaternion)


if __name__ == '__main__':

    import sys

    if len(sys.argv) < 2:
        print("<Usage> python main.py path_to_input_points.yaml")
        exit(1)

    INPUT_POINTS_PATH = sys.argv[1]
    room_1 = RoomOne(INPUT_POINTS_PATH)
    room_2 = RoomTwo(INPUT_POINTS_PATH)

    try:
        rospy.init_node('move_to_point', anonymous=True)
        navigator = GoToPose()

        room = room_1
        if moveBasedOnRedOrGreen("entrance", room):
            cI = colourIdentifier()
            while True:
                if cI.red_circle_detected:
                    rospy.loginfo("RED")
                    red = True
                    room = room_2
                    cI = None

                if moveBasedOnRedOrGreen("entrance", room):
                    cI2 = colourIdentifier()
                    if cI2.green_circle_detected:
                        rospy.loginfo("GREEN")
                        green = True
                        break
        else:
            exit(0)

        if green and moveBasedOnRedOrGreen("centre", room):
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
