from math import pi as PI

import numpy as np


def normalize_angle(angle):
    while angle > PI:
        angle -= 2 * PI
    while angle < -PI:
        angle += 2 * PI
    return angle


class Vector2D(object):
    # get vector by length and angle
    @staticmethod
    def create_by(length, angle):
        vector = Vector2D(length, 0)
        vector.data = vector.get_rotated(angle)

        return vector

    def __init__(self, x, y, data=None):
        if data is not None:
            self.data = data
        else:
            self.data = np.array([x, y], dtype='float64')

    def get_euclidean_distance(self, vec):
        # calculate euclidean distance between objects
        return np.linalg.norm(self.data - vec)

    # get around vector base rotated copy
    def get_rotated(self, angle):
        # create rotation matrix for robot angle
        c, s = np.cos(angle), np.sin(angle)
        R = np.array([[c, -s], [s, c]])

        return R.dot(self.data)

    def unit_vector(self):
        if self.data[0] == 0 and self.data[1] == 0:
            return None
        """ Returns the unit vector of the vector.  """
        return self.data / np.linalg.norm(self.data)

    def angle_between(self, v2):
        """
        compute angle (in degrees) for p0p1p2 corner
        Inputs:
            p0,p1,p2 - points in the form of [x,y]
        """

        v0 = v2.data
        v1 = self.data

        angle = np.math.atan2(np.linalg.det([v0, v1]), np.dot(v0, v1))
        return angle

        # v1_u = self.unit_vector()
        # v2_u = v2.unit_vector()
        # if v2_u is None or v1_u is None:
        #    return 0
        # if np.isnan(v1_u).any() or np.isnan(v2_u).any():
        #    return 0  # return an angle of zero if at least one zerovector
        # return normalize_angle(np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)))

    # get vector angle in radians
    def get_angle(self):
        return self.angle_between(Vector2D(1, 0))

    # get norm of the vector
    def get_length(self):
        return np.linalg.norm(self.data)