#! /usr/bin/env python

from os import path

import yaml


class GetCoordinates:

    """ Class to get the x,y coordinates of a particular rooms' centre or entrance. """

    def __init__(self, filepath):
        """ 
        Constructor

        :param filepath: Full path to the particular YAML file containing the coordinates.
        """
        filepath = path.expanduser(filepath)  # just in case the caller gave relative path.
        if not path.exists(filepath):
            raise EnvironmentError(filepath + " not found")

        self._filepath = filepath
        self._rooms = self._parse()

    def _parse(self):
        parsed = None
        with open(self._filepath, 'r') as fp:
            try:
                parsed = yaml.safe_load(fp)
            except (yaml.YAMLError, KeyError), err:
                raise KeyError("YAML parsing failed: " + str(err))
        return parsed

    def _get_points(self, room_number, points_location):
        """Private method to handle which room and which location of points to get."""
        # Handle parameter errors by caller
        if not isinstance(room_number, int):
            raise TypeError("room_number param must be specified as int.")

        # Check if caller wants room 1 or 2
        if room_number == 1:
            if points_location == "entrance":
                key = "room1_entrance_xy"
            else:
                # We default to centre if entrance wasn't specified
                key = "room1_centre_xy"
        elif room_number == 2:
            if points_location == "entrance":
                key = "room2_entrance_xy"
            else:
                key = "room2_centre_xy"
        else:
            raise ValueError("You specified an invalid room number, you passed: " + str(room_number))

        return self._rooms.get(key)

    def get_room_entrance(self, room_number):
        """
        Returns the x,y coordinates for a particular rooms' entrance.
        :param room_number: Which room.
        :type room_number: int
        :return: List containing the x,y coordinates for the specified room. Example x=1, y=3, return value: [1,3]
        """
        return self._get_points(room_number, "entrance")

    def get_room_centre(self, room_number):
        """
        Returns the x,y coordinates for a particular rooms' centre.

        :param room_number: Which room.
        :type room_number: int
        :return: List containing the x,y coordinates for the specified room. Example x=1, y=3, return value: [1,3]
        """
        return self._get_points(room_number, "centre")
