import pymel.core as pm
import maya.api.OpenMaya as om

from utils.vectors import average
from utils.vectors import sum_distance


class Limb(object):
    """ Functional API for a limb. """

    def __init__(self, name, startJoint=None, endJoint=None, parent=None):
        self.name = name

        selection = pm.ls(sl=True, type="joint")
        if startJoint and endJoint:
            self.joints = self.get_joints(startJoint, endJoint)
        elif len(selection) is 2:
            self.joints = self.get_joints(selection[0], selection[1])
        else:
            raise pm.error("No joints specified or selected, aborting operation.")

    def get_joints(self, s, e):
        ''' Attempt getting list of joints between joint s and joint e. '''
        jointList = []
        chain = None

        # Check joint s is in hierarchy of e
        if s.longName() in e.longName():
            chain = e.longName().split('|')
        elif e.longName() in s.longName():
            # User effed up, set s as e and vice versa...
            chain = s.longName().split('|')
            tmpS = s
            s = e
            e = s
        else:
            return jointList

        # Starting from the end joint we go up until we hit the start joint.
        for joint in reversed(chain):
            jointList.append(pm.ls(joint, head=1)[0])
            if joint == s.name():
                # Return a reversed list, so first item is the starting joint.
                return jointList[::-1]

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
            pm.joint(joint, e=True, rotationOrder=order)

    def orient(self, order = "yzx", up = "yup"):
        """ Modify the orientation for entire limb. Default to my personal preference. """
        # TODO Seeing we're using the standard MAYA implementation issues will occur... Investigate more stable options
        for joint in self.joints:
            if pm.listRelatives(joint, children = True, type ="joint"):
                pm.joint(joint, e=True, orientJoint=order, secondaryAxisOrient=up)
            else:
                # Zero out the last joint in the chain.
                pm.setAttr(joint.jointOrientX, 0)
                pm.setAttr(joint.jointOrientY, 0)
                pm.setAttr(joint.jointOrientZ, 0)

    def duplicate(self, prefix):
        # FIXME
        for joint in self.joints:
            pm.duplicate(joint, name=prefix + joint.name(), parentOnly=True)
            pm.parent(world=True)

    def save(self):
        """ Return a string to help in rebuilding same rig or reconnecting existing nodes. """
        # TODO Export a json showing which limb type, and which functionality been applied, ie is limb ik pvc or no flip.
        return self.__class__

    def pvc_ik(self, controller, distance = 1):
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

    def is_planar(self, tolerance = 0.001):
        """ Quick check to see if the Limb is planar"""
        # FIXME somehow not working as intended... not sure why though.
        points = [pm.datatypes.Point(pm.joint(x, q=True, p=True, a=True)) for x in self.joints]
        return points[0].planar(points, tolerance)

    def stretch(self, controller):
        """ Add Stretch to an IK limb. Possibly not belonging inside limb? Seeing it only applies to IK. """

        # TODO Seeing this is only implemented for IK limbs, maybe should be moved to a separate class for IK limbs.

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
            name = self.name + "_s_pointMatrixMult"
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
        multdivNode.setAttr("operation", 2)     # Set operation to "divide"

        pm.connectAttr(distanceNode.distance, conditionNode.firstTerm)
        conditionNode.setAttr("secondTerm", length)
        conditionNode.setAttr("operation", 2)   # Set operation to "Greater than"

        pm.connectAttr(multdivNode.outputX, conditionNode.colorIfTrueR)

        # TODO connect condition output to scale, or something...