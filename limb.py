import math

import pymel.core as pm
import maya.api.OpenMaya as om

from utils.vectors import average
from utils.vectors import sum_distance

""" Automate good structure, Limb requires or will create a Limb.name_component DAG object."""
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
        self.parent = parent

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
        # TODO, maybe solve using maya utility nodes
        points = [om.MPoint(pm.joint(x, q=True, p=True, a=True)) for x in self.joints]
        return sum_distance(points)

    def orient(self, aim=(1,0,0), up=(0,0,1), parent=None, child=None):
        """ """
        pass

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

        return copy

    def save(self):
        """ Return a string to help in rebuilding same rig or reconnecting existing nodes. """
        # TODO Export a json showing which limb type, and which functionality been applied, ie is limb ik pvc or no flip.
        return self.__class__

    def get_plane(self, mid=None):
        """ Create a plane based on position of first, last and mid average positions.
            :rtype: om.MPlane
        """

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

        # Might be a bit redundant, got the pymel.core.datatypes planar() and Point.planar() to work
        # >>> pm.datatypes.planar(*points)

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

        # TODO connect condition output to Translate of joints...

    def pvc_ik(self, controller, distance=1):
        """ Pole Vector IK """
        # TODO make controller optional, if None, create a srt buffer and controller object at end joint.
        vectors = [om.MVector(pm.joint(x, q=True, p=True, a=True)) for x in self.joints]

        mid = average([vectors[0], vectors[-1]])

        # Get an average position for all joints excluding first and last in list.
        avg = average(vectors[1:-1])

        # TODO, change method of getting position for PVC target.
        # Get the vector from midpoint to average point.
        direction = om.MVector([b - a for a, b in zip(mid, avg)])

        if distance:
            direction *= distance

        # Distance - will likely want a whole limb length as default...

        # Get position to place our PVC Target
        pvcTarget = om.MVector(avg + direction)

        # Create space locator to use as target.
        pvcLoc = pm.spaceLocator(p=pvcTarget, a=True)
        pm.xform(pvcLoc, centerPivots=True)

        ikHandle = pm.ikHandle(
            name='{}_ikHandle'.format(self.joints[-1].nodeName()),
            sj=self.joints[0],
            ee=self.joints[-1],
            solver='ikRPsolver'
        )[0]

        pm.poleVectorConstraint(pvcLoc, ikHandle)

        #pm.parent(ikHandle, controller)

    def slide(self):
        """ """
        pass

    def soft(self, controller):
        """ Create node network to set soft IK for limb."""

        # TODO: Review naming, and see if nodes could be removed.

        length = self.length()

        # Add attribute to our controller for soft IK damp factor.
        pm.addAttr(
            controller,
            shortName='sf',
            longName='soft',
            niceName='Soft',
            defaultValue=0.0,
            maxValue=1.0,
            minValue=0.0,
            hidden=False,
            keyable=True,
            readable=True,
        )

        # Decompose Matrices to get World Space Position for target and start of limb, then supply a distance node.
        translate_output = pm.group(empty=True, name='{}_translate_output'.format(self.name))
        pm.parent(translate_output, self.joints[0], relative=True)
        pm.parent(translate_output, self.joints[0].listRelatives(parent=True)[0].nodeName())

        start = pm.createNode('decomposeMatrix', name='{}_start_WMtx'.format(self.name))
        target = pm.createNode('decomposeMatrix', name='{}_target_WMtx'.format(self.name))

        targetDistance = pm.createNode('distanceBetween', name='{}_targetDistance'.format(self.name))

        # Connect sources to our decompose matrices
        pm.connectAttr(controller.worldMatrix[0], target.inputMatrix)
        pm.connectAttr(translate_output.worldMatrix[0], start.inputMatrix)

        # Hook the positions to the distance node, order shouldn't matter
        pm.connectAttr(target.outputTranslate, targetDistance.point1)
        pm.connectAttr(start.outputTranslate, targetDistance.point2)

        # Nodes for Soft IK solution.
        soft_distance = pm.createNode('multiplyDivide', name='{}_softDist'.format(self.name))
        soft_distance.setAttr('operation', 1)    # Set to Multiply
        soft_distance.setAttr('input2X', length)
        pm.connectAttr(controller.soft, soft_distance.input1X)

        # Hard Distance is the range we want IK to follow fully, which is total length - distance we want soft IK.
        hard_distance = pm.createNode('plusMinusAverage', name='{}_hardDist'.format(self.name))
        hard_distance.setAttr('operation', 2)    # Set to Subtract
        hard_distance.setAttr('input1D[0]', length)
        pm.connectAttr(soft_distance.outputX, hard_distance.input1D[1])

        # Get the Hard distance minus distance to target.
        subtract_distance = pm.createNode('plusMinusAverage', name='{}_subDist'.format(self.name))
        subtract_distance.setAttr('operation', 2)   # Subtract
        pm.connectAttr(hard_distance.output1D, subtract_distance.input1D[0])
        pm.connectAttr(targetDistance.distance, subtract_distance.input1D[1])

        distance_factor = pm.createNode('multiplyDivide', name='{}_distFac'.format(self.name))
        distance_factor.setAttr('operation', 2) # Divide
        pm.connectAttr(subtract_distance.output1D, distance_factor.input1X)
        pm.connectAttr(soft_distance.outputX, distance_factor.input2X)

        # To avoid divide by zero errors we also connect a condition to freeze division node.
        freeze_division = pm.createNode('condition', name='{}_freezeDivision'.format(self.name))
        freeze_division.setAttr('colorIfTrueR', 1)
        freeze_division.setAttr('colorIfFalseR', 0)
        freeze_division.setAttr('operation', 0)  # Equals
        freeze_division.setAttr('secondTerm', 0)
        pm.connectAttr(controller.soft, freeze_division.firstTerm)
        pm.connectAttr(freeze_division.outColorR, distance_factor.frozen)

        # Exponential function for our distance factor
        exp = pm.createNode('multiplyDivide', name='{}_exp'.format(self.name))
        exp.setAttr('operation', 3)  # Set to Power
        exp.setAttr('input1X', math.e)
        pm.connectAttr(distance_factor.outputX, exp.input2X)

        # 1 - result of exponential function.
        one_minus = pm.createNode('plusMinusAverage', name='{}_one_minus'.format(self.name))
        one_minus.setAttr('operation', 2)
        one_minus.setAttr('input1D[0]', 1)
        pm.connectAttr(exp.outputX, one_minus.input1D[1])

        # Multiply Soft Distance with the result of the exponential function.
        mult_soft_distance = pm.createNode('multiplyDivide', name='{}_multSoftExp'.format(self.name))
        mult_soft_distance.setAttr('operation', 1)  # Multiply
        pm.connectAttr(soft_distance.outputX, mult_soft_distance.input1X)
        pm.connectAttr(one_minus.output1D, mult_soft_distance.input2X)

        # Sum Hard Distance with whatever one can call the node above.
        final_soft_distance = pm.createNode('plusMinusAverage', name='{}_sum_softDistance'.format(self.name))
        final_soft_distance.setAttr('operation', 1) # Sum
        pm.connectAttr(hard_distance.output1D, final_soft_distance.input1D[0])
        pm.connectAttr(mult_soft_distance.outputX, final_soft_distance.input1D[1])

        # If Soft distance is greater than length.
        finalDist = pm.createNode('condition', name='{}_finalDistance'.format(self.name))
        finalDist.setAttr('operation', 5)  # Less or Equal
        pm.connectAttr(targetDistance.distance, finalDist.firstTerm)
        pm.connectAttr(hard_distance.output1D, finalDist.secondTerm)
        pm.connectAttr(targetDistance.distance, finalDist.colorIfTrueR)
        pm.connectAttr(final_soft_distance.output1D, finalDist.colorIfFalseR)

        soft_ratio = pm.createNode('multiplyDivide', name='{}_soft_ratio'.format(self.name))
        soft_ratio.setAttr('operation', 2)  # Multiply
        pm.connectAttr(finalDist.outColorR, soft_ratio.input1X)
        pm.connectAttr(targetDistance.distance, soft_ratio.input2X)

        # If soft ratio is not 0 soft ratio, else 1.
        ratio = pm.createNode('condition', name='{}_finalRatio'.format(self.name))
        ratio.setAttr('secondTerm', 0)
        ratio.setAttr('colorIfTrueR', 1)
        ratio.setAttr('operation', 0) # Equals
        pm.connectAttr(controller.soft, ratio.firstTerm)
        pm.connectAttr(soft_ratio.outputX, ratio.colorIfFalseR)

        # 1 - ratio to get what to multiply vectors with.
        subtract_ratio = pm.createNode('plusMinusAverage', name='{}_oneMinus_ratio'.format(self.name))
        subtract_ratio.setAttr('operation', 2)   # Subtract
        subtract_ratio.setAttr('input1D[0]', 1)
        pm.connectAttr(ratio.outColorR, subtract_ratio.input1D[1])

        # Start Vector * final ratio
        start_vector = pm.createNode('multiplyDivide', name='{}_sVector'.format(self.name))
        start_vector.setAttr('operation', 1)  # Multiply
        pm.connectAttr(start.outputTranslate, start_vector.input1)
        pm.connectAttr(subtract_ratio.output1D, start_vector.input2X)
        pm.connectAttr(subtract_ratio.output1D, start_vector.input2Y)
        pm.connectAttr(subtract_ratio.output1D, start_vector.input2Z)

        # Target Vector * final ratio
        target_vector = pm.createNode('multiplyDivide', name='{}_eVector'.format(self.name))
        target_vector.setAttr('operation', 1)  # Multiply
        pm.connectAttr(target.outputTranslate, target_vector.input1)
        pm.connectAttr(ratio.outColorR, target_vector.input2X)
        pm.connectAttr(ratio.outColorR, target_vector.input2Y)
        pm.connectAttr(ratio.outColorR, target_vector.input2Z)

        # Final World Position, sum of altered vectors.
        final_position = pm.createNode('plusMinusAverage', name='{}_sum_vectors'.format(self.name))
        final_position.setAttr('operation', 1)  # Sum
        pm.connectAttr(start_vector.output, final_position.input3D[0])
        pm.connectAttr(target_vector.output, final_position.input3D[1])

        # Compose Matrix
        compose_mtx = pm.createNode('composeMatrix', name='{}_final_ws_mtx'.format(self.name))
        pm.connectAttr(final_position.output3D, compose_mtx.inputTranslate)

        # Matrix Multiplication Final *
        multiply_mtx = pm.createNode('multMatrix', name='{}_cnst_mtx'.format(self.name))
        pm.connectAttr(compose_mtx.outputMatrix, multiply_mtx.matrixIn[0])

        # Create a null Transform object which we will apply resulting matrix onto.
        driven = pm.group(empty=True, name='{}_driven_srtBuffer'.format(self.name))
        pm.parent(driven, controller, relative=True)
        pm.parent(driven, world=True)
        pm.connectAttr(driven.parentInverseMatrix[0], multiply_mtx.matrixIn[1])

        # Driven decomposed matrix
        result_mtx = pm.createNode('decomposeMatrix', name='{}_offset_mtx'.format(self.name))
        pm.connectAttr(multiply_mtx.matrixSum, result_mtx.inputMatrix)
        pm.connectAttr(result_mtx.outputTranslate, driven.translate)

    def h_soft(self, controller):
        """ Soft IK as show on http://hhoughton07.wixsite.com/hazmondo/maya-ik-arm """

        # TODO not finished!

        """ Parent a Loc to top joint, use same parent as joint. 
        Get ws pos matrix for loc, vector loc->ctrl. Output soft will be magnitude for vector and applied to IK handle."""

        # Inputs:
        #   CTRL.Soft
        #   Limb.length
        #   Distance to CTRL

        # Outliner example
        # Component
        #   parent
        #       loc
        #       joints...
        #   ctrl
        #   ikHandle

        length = self.length()

        pm.addAttr(
            controller,
            shortName='sf',
            longName='soft',
            niceName='Soft',
            defaultValue=0.0,
            maxValue=1.0,
            minValue=0.0,
            hidden=False,
            keyable=True,
            readable=True,
        )

        # Create a locator, position at first joint in world, and set parent to same as for the joint.
        start = pm.spaceLocator(name='{}_softIK_start'.format(self.name))
        pm.parent(
            start,
            self.joints[0].listRelatives(parent=True),
            relative=True,
        )

        target = pm.createNode('decomposeMatrix', name='{}_target_WMtx')
        pm.connectAttr(controller.worldMatrix[0], target.inputMatrix)

        distance = pm.createNode('distanceBetween', name='{}_targetDistance')

        # Hook the positions to the distance node, order shouldn't matter
        pm.connectAttr(target.outputTranslate, distance.point1)
        pm.connectAttr(start.worldPosition[0], distance.point2)

        # Get Limb.length() minus controller.soft
        len = pm.createNode('plusMinusAverage', name='minLen')
        len.setAttr('operation', 2)
        len.setAttr('input1D[0]', length)
        pm.connectAttr(controller.soft, len.input1D[1])

        # Take the distance minus Soft attribute.
        dist = pm.createNode('plusMinusAverage', name='minDist')

        div = pm.createNode('multiplyDivide', name='divSoft_{}')
        div.setAttr('operation', 2)  # Set to Divide

        inv = pm.createNode('multiplyDivide', name='inv_')
        inv.setAttr('operation', 1) # Set to Multiply
        inv.setAttr('input1X', -1)
        pm.connectAttr(div.output, inv.input2)  # Only X channel carrying data.

        exp = pm.createNode('multiplyDivide', name='exp_{}'.format(self.name))
        exp.setAttr('operation', 3) # Set to Power
        exp.setAttr('input1X', math.e)
        pm.connectAttr(inv.output, exp.input2)

        mul = pm.createNode('multiplyDivide', name='multSoft')
        mul.setAttr('operation', 1) # Set to Multiply

        magnitude = pm.createNode('plusMinusAverage', name='minSoft')

        # If soft, soft factor, else limb length
        soft = pm.createNode('condition')

        # If distance less than length,
        final_magnitude = pm.createNode('condition')