import pymel.core as pymel
import maya.api.OpenMaya as om

from utils.vectors import average


class Limb(object):

    def __init__(self, name, startJoint=None, endJoint=None, parent=None):
        self.name = name

        selection = pymel.ls(sl=True, type="joint")
        if startJoint and endJoint:
            self.joints = self.get_joints(startJoint, endJoint)
        elif len(selection) is 2:
            self.joints = self.get_joints(selection[0], selection[1])
        else:
            raise pymel.error("No joints specified or selected, aborting operation.")

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
            jointList.append(pymel.ls(joint, head=1)[0])
            if joint == s.name():
                # Return a reversed list, so first item is the starting joint.
                return jointList[::-1]

    def toggle_local_rotation_axis(self):
        """ Toggle the Local Rotation Axis display. """
        pymel.general.toggle(self.joints, la=True)

    def set_rotation_order(self, order):
        """ Set the rotation order for each joint in the limb. """
        for joint in self.joints:
            pymel.joint(joint, e=True, rotationOrder=order)

    def orient(self, order = "yzx", up = "yup"):
        """ Modify the orientation for entire limb. Default to my personal preference. """
        for joint in self.joints:
            if pymel.listRelatives(joint, children = True, type = "joint"):
                pymel.joint(joint, e=True, orientJoint=order, secondaryAxisOrient=up)
            else:
                # Zero out the last joint in the chain.
                pymel.setAttr(joint.jointOrientX, 0)
                pymel.setAttr(joint.jointOrientY, 0)
                pymel.setAttr(joint.jointOrientZ, 0)

    def duplicate(self, prefix):
        # FIXME
        for joint in self.joints:
            pymel.duplicate(name=prefix + self.name, parentOnly=True)

    def save(self):
        """ Return a string to help in rebuilding same rig or reconnecting existing nodes. """
        # TODO Export a json showing which limb type, and which functionality been applied, ie is limb ik pvc or no flip.
        return self.__class__

    def pvc_ik(self, controller, distance = 1):
        """ Pole Vector IK """
        vectors = [om.MVector(pymel.joint(x, q=True, p=True, a=True)) for x in self.joints]

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
        pvcLoc = pymel.spaceLocator(p=pvcTarget, a=True)
        pymel.xform(pvcLoc, centerPivots=True)

        ikHandle = pymel.ikHandle(sj=self.joints[0], ee=self.joints[-1], solver='ikRPsolver')[0]
        pymel.poleVectorConstraint(pvcLoc, ikHandle)

        pymel.parent(ikHandle, controller)