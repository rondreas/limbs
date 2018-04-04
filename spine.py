import pymel.core as pm
import limb


class Spine(limb.Limb):

    def spline_ik(self):
        """ Setup a normal spline spine. """
        ikHandle, ikEffector, ikCurve = pm.ikHandle(
            name=self.name + "_ikh",
            startJoint=self.joints[0],
            endEffector=self.joints[-1],
            solver='ikSplineSolver',
            simplifyCurve=False
        )

        # Get the number of digits so we can set the zfill correctly,
        digits = len(str(len(ikCurve.cv)))

        # Iterate over each cv and create a cluster deformer,
        for i, cv in enumerate(ikCurve.cv):
            cluster_node, cluster_handle = pm.cluster(cv)
            cluster_handle.rename(
                ikCurve.nodeName() + '_ch_{}'.format(str(i).zfill(digits))
            )

    def stretch(self):

        pass


if __name__ == '__main__':
    pass
