import pymel.core as pm
import maya.api.OpenMaya as om

from utils.vectors import average
from utils.vectors import sum_distance


def get_joints(s, e):
    """ Get list of joints between joint s and e """
    # TODO single char variables are bad, figure something out.
    jointList = []

    # If joints were specified, use them.
    if type(s) is str and type(e) is str:
        s = pm.ls(s, type="joint")[0]
        e = pm.ls(e, type="joint")[0]

    # Check joint s is in hierarchy of e
    if s.longName() in e.longName():
        chain = e.longName().split('|')

    elif e.longName() in s.longName():
        # User effed up, set s as e and vice versa...
        chain = s.longName().split('|')
        s, e = e, s

    else:
        return jointList

    # Starting from the end joint we go up until we hit the start joint.
    for joint in reversed(chain):
        jointList.append(pm.ls(joint, head=1)[0])
        if joint == s.nodeName():
            # Return a reversed list, so first item is the starting joint.
            return jointList[::-1]

def getJointPosition(joint):
    """ Get absolute position of joint and return it as MPoint object. """
    return om.MVector(pm.joint(joint, query=True, position=True, absolute=True))

class Limb(object):
    """ Functional API for a limb. """

    def __init__(self, name, startJoint=None, endJoint=None, parent=None):
        self.name = name

        selection = pm.ls(sl=True, type="joint")
        if startJoint and endJoint:
            self.joints = get_joints(startJoint, endJoint)
        elif len(selection) is 2:
            self.joints = get_joints(selection[0], selection[1])
        else:
            # Is this the correct error to raise?
            raise pm.error("No joints specified or selected, aborting operation.")

    def length(self):
        """ Get length of limb. """
        points = [om.MPoint(pm.joint(x, q=True, p=True, a=True)) for x in self.joints]
        return sum_distance(points)

    def toggle_local_rotation_axis(self):
        """ Toggle the Local Rotation Axis display. """
        pm.general.toggle(self.joints, la=True)

    def set_rotation_order(self, order):
        """ Set the rotation order for each joint in the limb. """
        for joint in self.joints:
            joint.setAttr("rotateOrder", order)

    def duplicate(self, prefix):
        """ Duplicate the joints in limb, and return a new limb object for the copy."""

        # Duplicate all joints and ignore children.
        copy = pm.duplicate(self.joints, parentOnly=True)

        # Rename each copied joint to prefix + joint name
        for i, joint in enumerate(self.joints):
            copy[i].rename(prefix + joint.nodeName())

        # Parent new chain to world
        pm.parent(copy[0], world=True)

        return Limb(self.name + "_copy", copy[0], copy[-1])

    def save(self):
        """ Return a string to help in rebuilding same rig or reconnecting existing nodes. """
        # TODO Export a json showing which limb type, and which functionality been applied, ie is limb ik pvc or no flip.
        return self.__class__

    def get_plane(self, mid=None):
        """ Create a plane based on position of first, last and mid average positions.
            :rtype: om.MPlane
        """

        # TODO: Test rigorously - not sure getting the angle is good for deciding how to move the plane.

        # Get three points, start, end and mid so we can define a plane to check against
        startPoint = getJointPosition(self.joints[0])

        if isinstance(mid, pm.nodetypes.Joint):
            midPoint = getJointPosition(mid)
        else:
            # Get a list of all points but for start and end, then get their average position.
            midPoints = [getJointPosition(joint) for joint in self.joints[1:-1]]
            midPoint = average([om.MVector(p) for p in midPoints])

        endPoint = getJointPosition(self.joints[-1])

        # Get normal for our known points.
        normal = om.MVector(startPoint - midPoint) ^ om.MVector(midPoint - endPoint)

        # Define a plane from our normal at origin
        plane = om.MPlane()
        plane.setPlane(normal.normalize(), 0)

        # Get distance to a point known to be at the desired location.
        distance = plane.distanceToPoint(om.MVector(startPoint), signed=True)
        plane.setPlane(normal.normalize(), -distance)

        return plane

    def planar(self, tolerance=0.0001, mid=None):
        """ Check to see if limb joints are planar. """

        if len(self.joints) == 3:
            # Three points, yes we are planar why are you even asking?
            return True

        plane = self.get_plane(mid=mid)

        # For each joint, check distance to plane.
        for joint in self.joints[1:-1]:
            pos = om.MVector(getJointPosition(joint))
            if tolerance < plane.distanceToPoint(pos):
                # We consider this chain non planar and can call it a day already.
                return False

        # We managed to go through every joint and end up here so I guess the chain is planar.
        return True

    def make_planar(self, tolerance=0.0001, mid=None):
        """ Get a plane and move non-planar joints so chain is planar. """

        if len(self.joints) == 3:
            # Chain must be planar, do nothing.
            return True

        plane = self.get_plane(mid=mid)

        # Knowing first and last joint is on the plane, we can skip those two.
        for joint in self.joints[1:-1]:
            pos = om.MVector(getJointPosition(joint))
            if tolerance < plane.distanceToPoint(pos):
                distance = plane.distanceToPoint(pos, signed=True)
                new_pos = om.MVector(pos - om.MVector(plane.normal() * distance))
                pm.move(joint, new_pos, absolute=True, preserveChildPosition=True)

class IKLimb(Limb):

    def stretch(self, controller):
        """ Add Stretch to an IK limb. Possibly not belonging inside limb? Seeing it only applies to IK. """

        distanceNode = pm.createNode(
            "distanceBetween",
            name=self.name + "_distance"
        )

        multdivNode = pm.createNode(
            "multiplyDivide",
            name=self.name + "_multiplyDivide"
        )

        conditionNode = pm.createNode(
            "condition",
            name=self.name + "_condition"
        )

        startPointMatrixMult = pm.createNode(
            "pointMatrixMult",
            name=self.name + "_s_pointMatrixMult"
        )

        endPointMatrixMult = pm.createNode(
            "pointMatrixMult",
            name=self.name + "_e_pointMatrixMult"
        )

        length = self.length()

        # Get the world space position for first joint
        pm.connectAttr(self.joints[0].translate, startPointMatrixMult.inPoint)
        pm.connectAttr(self.joints[0].parentMatrix, startPointMatrixMult.inMatrix)

        pm.connectAttr(controller.translate, endPointMatrixMult.inPoint)
        pm.connectAttr(controller.parentMatrix, endPointMatrixMult.inMatrix)

        pm.connectAttr(startPointMatrixMult.output, distanceNode.point1)
        pm.connectAttr(endPointMatrixMult.output, distanceNode.point2)

        pm.connectAttr(distanceNode.distance, multdivNode.input1X)
        multdivNode.setAttr("input2X", length)
        multdivNode.setAttr("operation", 2)  # Set operation to "divide"

        pm.connectAttr(distanceNode.distance, conditionNode.firstTerm)
        conditionNode.setAttr("secondTerm", length)
        conditionNode.setAttr("operation", 2)  # Set operation to "Greater than"

        pm.connectAttr(multdivNode.outputX, conditionNode.colorIfTrueR)

        # TODO connect condition output to scale, or something...

    def pvc_ik(self, controller, distance=1):
        """ Pole Vector IK """
        vectors = [om.MVector(pm.joint(x, q=True, p=True, a=True)) for x in self.joints]

        mid = average([vectors[0], vectors[-1]])

        # Get an average position for all joints excluding first and last in list.
        avg = average(vectors[1:-1])

        # Get the vector from midpoint to average point.
        direction = om.MVector([b - a for a, b in zip(mid, avg)])

        if distance:
            direction *= distance

        # Get position to place our PVC Target
        pvcTarget = om.MVector(avg + direction)

        # Create space locator to use as target.
        pvcLoc = pm.spaceLocator(p=pvcTarget, a=True)
        pm.xform(pvcLoc, centerPivots=True)

        ikHandle = pm.ikHandle(sj=self.joints[0], ee=self.joints[-1], solver='ikRPsolver')[0]
        pm.poleVectorConstraint(pvcLoc, ikHandle)

        pm.parent(ikHandle, controller)