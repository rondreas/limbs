import pymel.core as pm
import limb

class Leg(limb.Limb):

    def no_flip_ik(self, controller):
        """ """

        ikHandle = pm.ikHandle(
            name = self.name + "_ikHandle",
            startJoint=self.joints[0],
            endEffector=self.joints[-1],
            solver='ikRPsolver'
        )[0]

        pm.setAttr(ikHandle.poleVectorX, 0.1)
        pm.setAttr(ikHandle.poleVectorY, 0)
        pm.setAttr(ikHandle.poleVectorZ, 0)

        pm.parent(ikHandle, controller)

        # Create custom attributes for controller object to control knee twist.
        pm.addAttr(
            controller,
            shortName='ko',
            longName='kneeOffset',
            defaultValue=90.0,
            hidden=False,
            readable=True,
        )

        pm.addAttr(
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
        kneeAdd = pm.createNode(
            'plusMinusAverage',
            name=self.name + '_avgNode'
        )

        pm.connectAttr(controller.kneeOffset, kneeAdd.input1D[0])
        pm.connectAttr(controller.kneeTwist, kneeAdd.input1D[1])
        pm.connectAttr(kneeAdd.output1D, ikHandle.twist)
