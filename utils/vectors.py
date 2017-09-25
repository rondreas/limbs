import maya.api.OpenMaya as om


def average(vectors):
    ''' Return the average vector for all vectors. '''
    return om.MVector([sum(v) / len(vectors) for v in zip(*vectors)])