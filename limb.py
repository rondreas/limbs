import math

import pymel.core as pm
import maya.api.OpenMaya as om

import controllers.controller as ctrl
import utils.vectors


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


def get_joint_position(joint):
    """ Get absolute position of joint and return it as MPoint object. """
    return om.MVector(pm.joint(joint, query=True, position=True, absolute=True))


class Limb(object):
    """ Functional API for a limb. """

    def __init__(self, name, startJoint=None, endJoint=None):

        self.name = name

        selection = pm.ls(sl=True, type="joint")
        if startJoint and endJoint:
            self.joints = get_joints(startJoint, endJoint)
        elif len(selection) is 2:
            self.joints = get_joints(selection[0], selection[1])
        else:
            # Is this the correct error to raise?
            raise pm.error("No joints specified or selected, aborting operation.")

        if not self.joints:
            raise ValueError("Limb contains no joints. ")

        self.children = self.joints[-1].getChildren()

    def __getitem__(self, item):
        return self.joints[item]

    def __repr__(self):
        return 'Limb({name}, {startJoint}, {endJoint})'.format(name=self.name, startJoint=self[0], endJoint=self[-1])

    def length(self):
        """ Get length of limb. """
        points = [om.MPoint(pm.joint(x, q=True, p=True, a=True)) for x in self.joints]
        return utils.vectors.sum_distance(points)

    def associate(self):
        """ Create Message attributes for each joint, and connect to all other joints. """

        # Create compound attribute to store hierarchy connection in,
        for joint in self.joints:

            # If already present, delete the attribute to recreate it,
            if joint.nodeName()+'.limb' in joint.listAttr():
                pm.deleteAttr(joint, attribute='limb')

            # Create the attribute, make sure top and end has size 1,
            pm.addAttr(
                joint,
                longName='limb',
                attributeType='compound',
                numberOfChildren=1 if (joint == self.joints[0] or joint == self.joints[-1]) else 2,
            )

        # Create attributes to connect parent child onto,
        for parent, child in zip(self.joints[:-1], self.joints[1:]):
            pm.addAttr(child, longName='limbParent', attributeType='message', parent='limb')
            pm.addAttr(parent, longName='limbChild', attributeType='message', parent='limb')

        # Make connections, apparently can't make it work inside same loop as creation.
        for parent, child in zip(self.joints[:-1], self.joints[1:]):
            pm.connectAttr(parent.message, child.limbParent)
            pm.connectAttr(child.message, parent.limbChild)

    def orient(self):
        """ Set orientation for limb,

        blatantly taken method from Pedro Bellini show here:
         http://lesterbanks.com/2014/04/creating-aim-transformation-maya-python-api/

        """

        # TODO; avoid makeIdentity and solve using math.

        # Parent everything but top joint to world,
        pm.parent(self.joints[1:], world=True)
        pm.parent(self.children, world=True)

        # Get vectors for each joint to it's child in limb,
        aim_vectors = list()
        for index in range(len(self.joints[:-1])):
            p1 = self.joints[index].getAttr('worldMatrix').translate
            p2 = self.joints[index + 1].getAttr('worldMatrix').translate
            vector = (p2 - p1).normal()
            aim_vectors.append(vector)

        # For every two vectors we will get an up vectors from their cross product,
        up_vectors = list()
        for v, w in zip(aim_vectors[:-1], aim_vectors[1:]):
            up_vectors.append((v ^ w).normal())

        # Extend the vector arrays,
        aim_vectors.append(aim_vectors[-1])
        up_vectors.insert(0, up_vectors[0])
        up_vectors.append(up_vectors[-1])

        # For each joint, it's desired aim direction and up direction.
        for joint, aim, up in zip(self.joints, aim_vectors, up_vectors):

            # Zero out rotation and orient values for joint,
            pm.makeIdentity(
                joint,
                apply=True,
                jointOrient=True,
                rotate=True,
            )

            # Define axis which are to be rotated into position,
            aim_axis = pm.dt.Vector.xAxis
            up_axis = pm.dt.Vector.zNegAxis
            side = (aim ^ up).normal()

            # Re-evaluate the up vector by taking the cross product of side and aim to get orthogonal vectors,
            up = (side ^ aim).normal()

            # Quaternion U describing the rotation to get from aim axis to aim vector,
            quat_u = pm.dt.Quaternion(aim_axis, aim)

            # Rotate the up axis with
            up_rotated = up_axis.rotateBy(quat_u)

            # Get the angle in radians between up vector and the rotated up vector,
            angle = math.acos(up_rotated * up)

            # Get Quaternion V describing angle radians around aim vector,
            quat_v = pm.dt.Quaternion(angle, aim)

            # Check rotation didn't go wrong way,
            if not up.isEquivalent(up_rotated.rotateBy(quat_v), 1.0e-5):
                angle = (2 * math.pi) - angle
                quat_v = pm.dt.Quaternion(angle, aim)

            # Set the final rotation Quaternion, and multiply the inverse of original orientation,
            final_rotation = quat_u * quat_v
            joint.setRotation(final_rotation)

            # Any rotation values will be baked into jointOrient attribute,
            pm.makeIdentity(
                joint,
                apply=True,
                r=True,
            )

        # Re-parent children to previous parent,
        for parent, child in zip(self.joints[:-1], self.joints[1:]):
            pm.parent(child, parent)

        # And any orphan children as well.
        for child in self.children:
            pm.parent(child, self.joints[-1])

    def toggle_local_rotation_axis(self):
        """ Toggle the Local Rotation Axis display. """
        pm.general.toggle(self.joints, la=True)

    def set_rotation_order(self, order):
        """ Set the rotation order for each joint in the limb. """
        for joint in self.joints:
            joint.setAttr("rotateOrder", order)

    def duplicate(self, prefix='', suffix='', parent=None):
        """ Duplicate joints in limb, optionally adding prefix and suffix to name, or replacing a substring.
        Then parenting the top node to specified parent or world.

        :argument: prefix - a string to add before the name of copied joint,
        :argument: suffix - a string to add after the name of copied joint,
        :argument: parent - a string specifying a node, or a pymel node object,

        :returns: list of pymel joints

        """

        # Duplicate all joints and ignore children.
        copy = pm.duplicate(self.joints, parentOnly=True)

        for old, new in zip(self.joints, copy):
            name = old.nodeName()
            new.rename(prefix + name + suffix)

        if parent:
            pm.parent(copy[0], parent)

        else:
            pm.parent(copy[0], world=True)

        return copy

    def save(self):
        """ Return a string to help in rebuilding same rig or reconnecting existing nodes. """
        # TODO Export a json showing which limb type, and which functionality been applied.
        return self.__class__

    def get_plane(self, mid=None):
        """ Create a plane based on position of first, last and mid average positions.
            :rtype: om.MPlane
        """

        # Get three points, start, end and mid so we can define a plane to check against
        startPoint = get_joint_position(self.joints[0])

        if isinstance(mid, pm.nodetypes.Joint):
            midPoint = get_joint_position(mid)
        else:
            # Get a list of all points but for start and end, then get their average position.
            midPoints = [get_joint_position(joint) for joint in self.joints[1:-1]]
            midPoint = utils.vectors.average([om.MVector(p) for p in midPoints])

        endPoint = get_joint_position(self.joints[-1])

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
            pos = om.MVector(get_joint_position(joint))
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
            pos = om.MVector(get_joint_position(joint))
            if tolerance < plane.distanceToPoint(pos):
                distance = plane.distanceToPoint(pos, signed=True)
                new_pos = om.MVector(pos - om.MVector(plane.normal() * distance))
                pm.move(joint, new_pos, absolute=True, preserveChildPosition=True)


class IKLimb(Limb):

    def __init__(self, name, startJoint=None, endJoint=None, parent=None):
        super(IKLimb, self).__init__(name, startJoint, endJoint)
        self.ikHandle = None

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

    def pvc_ik(self, controller, pvc_target=None):
        """ Pole Vector IK """

        # Parameter validation
        if not isinstance(controller, pm.nodetypes.Transform):
            raise ValueError("Bad data type for parameter 'controller'")

        if pvc_target and not isinstance(pvc_target, pm.nodetypes.Transform):
            raise ValueError("Bad data type for parameter 'pvc_target'")

        # Get position vectors for all joints in limb
        vectors = [om.MVector(pm.joint(x, q=True, p=True, a=True)) for x in self.joints]

        # Define a line between first and last joint
        line = utils.vectors.Line(vectors[0], vectors[-1])

        # Get an average position for all joints excluding first and last in list.
        avg = utils.vectors.average(vectors[1:-1])

        # Get world position along line closest to avg vector.
        mid = line.closest_point_along_line_to(avg)
        print("Mid Position: {}".format(mid))
        print("Avg Position: {}".format(avg))

        # Get a direction along which to place pvc target.
        direction = om.MVector(avg - mid).normal()
        print("Dir Pre: {}".format(direction))

        # Add magnitude to vector, either closest distance to our line, or length of our limb.
        if pvc_target:
            direction *= line.distance_to(pm.xform(pvc_target, q=True, ws=True, t=True))
        else:
            direction *= self.length()

        print("Dir Post: {}".format(direction))
        # TODO Debug Position, am I starting from first joint?
        # Get position to place our PVC Target
        position = om.MVector(avg + direction)

        print("Position: {}".format(position))

        if pvc_target:
            # Position the pvc target controller,
            pm.xform(
                pvc_target,
                ws=True,
                t=position
            )
            # Create the srt buffer group to zero out the controller
            ctrl.srt_buffer(target=pvc_target, child=pvc_target)

        else:
            pvc_target = pm.spaceLocator(p=position, a=True, name="{}_pvc_target".format(self.name))
            pm.xform(pvc_target, centerPivots=True)

        self.ikHandle = pm.ikHandle(
            name='{}_ikHandle'.format(self.joints[-1].nodeName()),
            sj=self.joints[0],
            ee=self.joints[-1],
            solver='ikRPsolver'
        )[0]

        pm.poleVectorConstraint(pvc_target, self.ikHandle)
        pm.parent(self.ikHandle, controller)

    def slide(self):
        """ """
        pass

    def soft(self, controller):
        """ Create node network to set soft IK for limb."""

        # TODO Out Vector = Length if distance to ctrl is equal to or less than length...

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
        soft_distance.setAttr('operation', 1)  # Set to Multiply
        soft_distance.setAttr('input2X', length)
        pm.connectAttr(controller.soft, soft_distance.input1X)

        # Hard Distance is the range we want IK to follow fully, which is total length - distance we want soft IK.
        hard_distance = pm.createNode('plusMinusAverage', name='{}_hardDist'.format(self.name))
        hard_distance.setAttr('operation', 2)  # Set to Subtract
        hard_distance.setAttr('input1D[0]', length)
        pm.connectAttr(soft_distance.outputX, hard_distance.input1D[1])

        # Get the Hard distance minus distance to target.
        subtract_distance = pm.createNode('plusMinusAverage', name='{}_subDist'.format(self.name))
        subtract_distance.setAttr('operation', 2)  # Subtract
        pm.connectAttr(hard_distance.output1D, subtract_distance.input1D[0])
        pm.connectAttr(targetDistance.distance, subtract_distance.input1D[1])

        distance_factor = pm.createNode('multiplyDivide', name='{}_distFac'.format(self.name))
        distance_factor.setAttr('operation', 2)  # Divide
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
        final_soft_distance.setAttr('operation', 1)  # Sum
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
        ratio.setAttr('operation', 0)  # Equals
        pm.connectAttr(controller.soft, ratio.firstTerm)
        pm.connectAttr(soft_ratio.outputX, ratio.colorIfFalseR)

        # 1 - ratio to get what to multiply vectors with.
        subtract_ratio = pm.createNode('plusMinusAverage', name='{}_oneMinus_ratio'.format(self.name))
        subtract_ratio.setAttr('operation', 2)  # Subtract
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

        # TODO temporary fix make something better...
        pm.parent(self.ikHandle, driven)

    def h_soft(self, controller, pvc_target=None):
        """ Soft IK as show on http://hhoughton07.wixsite.com/hazmondo/maya-ik-arm """

        # if no IK implemented so run the regular pole vector IK.
        if not self.ikHandle:
            self.pvc_ik(controller, pvc_target)

        # Get length of limb
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

        # Create a group at top joint, so we got a source for position in order to avoid cycle checks.
        translate_output = pm.group(empty=True, name='{}_translate_output'.format(self.name))
        pm.parent(translate_output, self.joints[0], relative=True)
        pm.parent(translate_output, self.joints[0].listRelatives(parent=True)[0].nodeName())

        # Get matrices for start and controller position
        start = pm.createNode('decomposeMatrix', name='{}_start_WMtx'.format(self.name))
        target = pm.createNode('decomposeMatrix', name='{}_target_WMtx'.format(self.name))

        pm.connectAttr(translate_output.worldMatrix[0], start.inputMatrix)
        pm.connectAttr(controller.worldMatrix[0], target.inputMatrix)

        distance = pm.createNode('distanceBetween', name='{}_targetDistance'.format(self.name))

        # Hook the world positions to the distance node, order shouldn't matter
        pm.connectAttr(target.outputTranslate, distance.point1)
        pm.connectAttr(start.outputTranslate, distance.point2)

        # Get Limb.length() minus controller.soft
        len = pm.createNode('plusMinusAverage', name='{}_minLen'.format(self.name))
        len.setAttr('operation', 2)  # Subtract
        len.setAttr('input1D[0]', length)
        pm.connectAttr(controller.soft, len.input1D[1])

        # Take the distance minus Soft attribute.
        dist = pm.createNode('plusMinusAverage', name='{}_minDist'.format(self.name))
        dist.setAttr('operation', 2)  # Subtract
        pm.connectAttr(distance.distance, dist.input1D[0])
        pm.connectAttr(len.output1D, dist.input1D[1])

        # distance-(length-soft)/soft
        div = pm.createNode('multiplyDivide', name='{}_divSoft'.format(self.name))
        div.setAttr('operation', 2)  # Set to Divide
        pm.connectAttr(dist.output1D, div.input1X)
        pm.connectAttr(controller.soft, div.input2X)

        inv = pm.createNode('multiplyDivide', name='{}_inv'.format(self.name))
        inv.setAttr('operation', 1)  # Set to Multiply
        inv.setAttr('input1X', -1)
        pm.connectAttr(div.output, inv.input2)  # Only X channel carrying data.

        exp = pm.createNode('multiplyDivide', name='{}_exp'.format(self.name))
        exp.setAttr('operation', 3)  # Set to Power
        exp.setAttr('input1X', math.e)
        pm.connectAttr(inv.output, exp.input2)

        mul = pm.createNode('multiplyDivide', name='{}_multSoft'.format(self.name))
        mul.setAttr('operation', 1)  # Set to Multiply
        pm.connectAttr(controller.soft, mul.input1X)
        pm.connectAttr(exp.outputX, mul.input2X)

        magnitude = pm.createNode('plusMinusAverage', name='{}_magnitude'.format(self.name))
        magnitude.setAttr('operation', 2)  # Subtract
        magnitude.setAttr('input1D[0]', length)
        pm.connectAttr(mul.outputX, magnitude.input1D[1])

        # If soft, soft factor, else limb length
        soft = pm.createNode('condition', name='{}_ifSoft'.format(self.name))
        soft.setAttr('operation', 2)  # Greater than
        soft.setAttr('secondTerm', 0)
        soft.setAttr('colorIfFalseR', length)
        pm.connectAttr(magnitude.output1D, soft.colorIfTrueR)
        pm.connectAttr(controller.soft, soft.firstTerm)

        # If distance greater than length,
        final_magnitude = pm.createNode('condition', name='{}_distGreaterLength'.format(self.name))
        final_magnitude.setAttr('operation', 2)  # Greater than
        pm.connectAttr(distance.distance, final_magnitude.colorIfFalseR)
        pm.connectAttr(soft.outColorR, final_magnitude.colorIfTrueR)
        pm.connectAttr(distance.distance, final_magnitude.firstTerm)
        pm.connectAttr(len.output1D, final_magnitude.secondTerm)

        # Target - Start =  vector(start->target)
        direction = pm.createNode('plusMinusAverage', name='{}_dirVector'.format(self.name))
        direction.setAttr('operation', 2)  # Subtract
        pm.connectAttr(target.outputTranslate, direction.input3D[0])
        pm.connectAttr(start.outputTranslate, direction.input3D[1])

        # Normalize vector so we only get direction.
        normalized = pm.createNode('vectorProduct', name='{}_normalized'.format(direction.nodeName()))
        normalized.setAttr('operation', 0)  # No Operation
        normalized.setAttr('normalizeOutput', 1)  # Normalize the output
        pm.connectAttr(direction.output3D, normalized.input1)

        # Multiply the magnitude with the normalized directional vector.
        final_vector = pm.createNode('multiplyDivide', name='{}_softIK_output'.format(self.name))
        final_vector.setAttr('operation', 1)  # Multiply
        # Connect all the scalar inputs X, Y, Z
        pm.connectAttr(final_magnitude.outColorR, final_vector.input2X)
        pm.connectAttr(final_magnitude.outColorR, final_vector.input2Y)
        pm.connectAttr(final_magnitude.outColorR, final_vector.input2Z)
        # Connect the Vector
        pm.connectAttr(normalized.output, final_vector.input1)

        # Add Position Vector for top joint, and the output from soft IK solution.
        ik_world_position = pm.createNode('plusMinusAverage', name='{}_target_WPos'.format(self.ikHandle.nodeName()))
        ik_world_position.setAttr('operation', 1)  # Sum
        pm.connectAttr(start.outputTranslate, ik_world_position.input3D[0])
        pm.connectAttr(final_vector.output, ik_world_position.input3D[1])

        # TODO Sort the hierarchy for ikHandle, and give ik_world_position.output3D as worldspace transforms.

        # Temporary Fix
        pm.parent(self.ikHandle, world=True)
        pm.connectAttr(
            ik_world_position.output3D,
            self.ikHandle.translate
        )
