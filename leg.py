import pymel.core as pymel
import limb

class Leg(limb.Limb):

    def no_flip_ik(self, controller):
        """ """

        ikHandle = pymel.ikHandle(
            startJoint=self.joints[0],
            endEffector=self.joints[-1],
            solver='ikRPsolver'
        )[0]

        pymel.setAttr(ikHandle.poleVectorX, 0.1)
        pymel.setAttr(ikHandle.poleVectorY, 0)
        pymel.setAttr(ikHandle.poleVectorZ, 0)

        pymel.parent(ikHandle, controller)

        # Create custom attributes for controller object to control knee twist.
        pymel.addAttr(
            controller,
            shortName='ko',
            longName='kneeOffset',
            defaultValue=90.0,
            hidden=False,
            readable=True,
        )

        pymel.addAttr(
            controller,
            shortName='kt',
            longName='kneeTwist',
            defaultValue=0.0,
            hidden=False,
            keyable=True,
            readable=True,
        )

        # Create a utility node to get sum of custom attributes.
        # and add output to ik twist.
        kneeAdd = pymel.createNode(
            'plusMinusAverage',
            name=self.name + '_avgNode'
        )

        pymel.connectAttr(controller.kneeOffset, kneeAdd.input1D[0])
        pymel.connectAttr(controller.kneeTwist, kneeAdd.input1D[1])
        pymel.connectAttr(kneeAdd.output1D, ikHandle.twist)
