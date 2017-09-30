import maya.api.OpenMaya as om


def average(vectors):
    """ Return the average vector for all vectors. """
    return om.MVector([sum(v) / len(vectors) for v in zip(*vectors)])

def sum_distance(points):
    """ Return the sum of distances starting from p1 -> pN"""
    totalDistance = 0
    for p in range(len(points) - 1):
        totalDistance += points[p].distanceTo(points[p+1])
    return totalDistance