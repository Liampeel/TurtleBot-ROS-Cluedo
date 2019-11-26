from getcoordinates import GetCoordinates


class RoomFactory:

    """ This class shouldn't be called directly, instead call one of the derived classes. """

    def __init__(self, filepath, room_number):
        coordinates = GetCoordinates(filepath)

        self._centre = coordinates.get_room_centre(room_number)
        self._entrance = coordinates.get_room_entrance(room_number)

    def get_centre(self, coordinate=None):
        """
        Returns points for the centre coordinates of a room.

        :param coordinate: "x", "y" or None. If None then a list is returned of the form [x,y].
        """
        if coordinate == "x":
            return self._centre[0]
        elif coordinate == "y":
            return self._centre[1]

        return self._centre

    def get_entrance(self, coordinate=None):
        """
        Returns points for the entrance coordinates of a room.

        :param coordinate: "x", "y" or None. If None then a list is returned of the form [x,y].
        """
        if coordinate == "x":
            return self._entrance[0]
        elif coordinate == "y":
            return self._entrance[1]

        return self._entrance


class RoomOne(RoomFactory):

    """ Class for representing points in Room One """

    def __init__(self, input_points_filepath):
        RoomFactory.__init__(self, input_points_filepath, 1)


class RoomTwo(RoomFactory):

    """ Class for representing points in Room Two """

    def __init__(self, input_points_filepath):
        RoomFactory.__init__(self, input_points_filepath, 2)
