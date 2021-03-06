import maya.api.OpenMaya as om
import math


def average(vectors):
    """ Return the average vector for all vectors. """
    return om.MVector([sum(v) / len(vectors) for v in zip(*vectors)])

def sum_distance(points):
    """ Return the sum of distances starting from p1 -> pN"""
    totalDistance = 0
    for p in range(len(points) - 1):
        totalDistance += points[p].distanceTo(points[p+1])
    return totalDistance

class Line(object):
    def __init__(self, pointA, pointB):
        self.a = om.MPoint(pointA)
        self.b = om.MPoint(pointB)

    def distance_to(self, point):
        # Make point into a Maya point object
        p = om.MPoint(point)
        # Calculate distance d from line to point p
        d = (om.MVector(p - self.a) ^ om.MVector(p - self.b)).length() / om.MVector(self.b - self.a).length()
        return d

    def closest_point_along_line_to(self, point):
        """ Solve the position vector for the point along line which is closest to given point. """

        c = om.MPoint(point)

        hypotenuse = om.MVector(self.a - c).length()
        cathetus =  self.distance_to(point)
        direction = om.MVector(self.b - self.a).normal()
        position = direction * math.sqrt(hypotenuse**2 - cathetus**2)

        if om.MVector(self.b - self.a) * om.MVector(self.a - c) > 0:
            return -1 * position + om.MVector(self.a)

        else:
            return position + om.MVector(self.a)