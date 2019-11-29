#! /usr/bin/env python

import rospy
import numpy as np
from room import RoomOne, RoomTwo
from go_to_specific_point_on_map import GoToPose
from CircleDetection import colourIdentifier

if __name__ == '__main__':

    # TODO: We might be better off getting the user to specify where the input_points.yaml file is.

    # Get all the coordinates for room 1 & 2
    INPUT_POINTS_PATH = "../example/input_points.yaml"
    room_1 = RoomOne(INPUT_POINTS_PATH)
    room_2 = RoomTwo(INPUT_POINTS_PATH)
    green = False
    red = False

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

            while True:
                cI=colourIdentifier()
                if cI.green_circle_detected:
                    rospy.loginfo("GREEN")
                    green = True
                    break
                elif cI.red_circle_detected:
                    rospy.loginfo("RED")
                    red = True
                    break
            rospy.loginfo("DETECTED ONE")
        else:
            rospy.loginfo("The base failed to reach the desired pose")

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
            if success:
                rospy.loginfo("Hooray, reached the desired pose")
            else:
                rospy.loginfo("The base failed to reach the desired pose")

        # Sleep to give the last log messages time to be sent
        rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")
