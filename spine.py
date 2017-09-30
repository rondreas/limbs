import pymel.core as pm
import limb


class Spine(limb.Limb):

    def spline_ik(self):
        """ Setup a normal spline spine. """
        # TODO add input variable for number of controllers, and create the number of controllers specified.
        ikHandle, ikEffector, ikCurve = pm.ikHandle(
            name=self.name + "_ikHandle",
            startJoint=self.joints[0],
            endEffector=self.joints[-1],
            solver='ikSplineSolver',
            simplifyCurve=False
        )

        for cv in ikCurve.cv:
            pm.cluster(cv.name(), name=cv.name() + '_cluster')

    def ribbon(self):
        """ Setup a ribbon spine. """
        pass


if __name__ == '__main__':
    pass
