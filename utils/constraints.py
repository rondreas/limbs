import pymel.core as pm


# TODO Make into objects

def parent(source, target, name='', maintainOffset=True):
    """ Creates a parent constrain using maya nodes.
        Thanks Cult of Rig and http://bindpose.com.

        example usage:
        >>> from limbs.utils import constraints
        >>> import pymel.core as pm
        >>> src, trgt = pm.selected()
        >>> constraints.parent(src, trgt, name='parentConstraint')
        """

    # Might suffer from crazy long node names. We will see...
    name = name if name else source.nodeName() + target.nodeName()

    # Create required nodes,
    multMtx = pm.createNode('multMatrix', name='{}_multMtx'.format(name))
    decompMtx = pm.createNode('decomposeMatrix', name='{}_decompMtx'.format(name))

    if maintainOffset:
        offsetMtx = pm.createNode('multMatrix', name='{}_offsetMtx'.format(name))
        pm.connectAttr(target.worldMatrix[0], offsetMtx.matrixIn[0])
        pm.connectAttr(source.worldInverseMatrix[0], offsetMtx.matrixIn[1])
        pm.setAttr(multMtx.matrixIn[0], offsetMtx.getAttr('matrixSum'))

    # Assign connections
    pm.connectAttr(source.worldMatrix[0], multMtx.matrixIn[1])
    pm.connectAttr(target.parentInverseMatrix[0], multMtx.matrixIn[2])

    pm.connectAttr(multMtx.matrixSum, decompMtx.inputMatrix)

    pm.connectAttr(decompMtx.outputTranslate, target.translate)
    pm.connectAttr(decompMtx.outputRotate, target.rotate)


def aim(source, target, name=''):
    """ Create an aim constraint with maya nodes. """

    # Works, world up check

    source_decomposed = pm.createNode('decomposeMatrix', name='{}_dcmpMtx'.format(source.nodeName()))
    pm.connectAttr(source.worldMatrix[0], source_decomposed.inputMatrix)

    target_decomposed = pm.createNode('decomposeMatrix', name='{}_dcmpMtx'.format(target.nodeName()))
    pm.connectAttr(target.worldMatrix[0], target_decomposed.inputMatrix)

    # Get the aim vector, target - source position.
    constraint_vector = pm.createNode('plusMinusAverage', name='{}_constraintVector'.format(name))
    constraint_vector.setAttr('operation', 2)   # Subtraction
    pm.connectAttr(target_decomposed.outputTranslate, constraint_vector.input3D[0])
    pm.connectAttr(source_decomposed.outputTranslate, constraint_vector.input3D[1])

    # Normalize the aim vector
    aim_vector = pm.createNode('vectorProduct', name='{}_aimVector'.format(name))
    aim_vector.setAttr('normalizeOutput', True)
    aim_vector.setAttr('operation', 0)
    pm.connectAttr(constraint_vector.output3D, aim_vector.input1)

    # side vector = aim ^ up = aim cross up
    side_vector = pm.createNode('vectorProduct', name='{}_sideVector'.format(name))
    side_vector.setAttr('normalizeOutput', True)
    side_vector.setAttr('operation', 2)
    side_vector.setAttr('input2', (0,1,0))
    pm.connectAttr(aim_vector.output, side_vector.input1)

    up_vector = pm.createNode('vectorProduct', name='{}_upVector'.format(name))
    up_vector.setAttr('normalizeOutput', True)
    up_vector.setAttr('operation', 2)
    pm.connectAttr(side_vector.output, up_vector.input1)
    pm.connectAttr(aim_vector.output, up_vector.input2)

    """ 
    Depending on which axis we want aiming one can plug the vectors into the 4x4 mtx differently, 
    for now however we will assume X-aim and Y-up
    """
    mtx = pm.createNode('fourByFourMatrix', name='{}_4x4Mtx'.format(name))

    pm.connectAttr(side_vector.outputX, mtx.in00)
    pm.connectAttr(side_vector.outputY, mtx.in01)
    pm.connectAttr(side_vector.outputZ, mtx.in02)

    pm.connectAttr(aim_vector.outputX, mtx.in10)
    pm.connectAttr(aim_vector.outputY, mtx.in11)
    pm.connectAttr(aim_vector.outputZ, mtx.in12)

    pm.connectAttr(up_vector.outputX, mtx.in20)
    pm.connectAttr(up_vector.outputY, mtx.in21)
    pm.connectAttr(up_vector.outputZ, mtx.in22)

    # Finally a decompose matrix to get the resulting rotation from.
    decomposeMtx = pm.createNode('decomposeMatrix', name='{}_dcmpMtx'.format(name))
    pm.connectAttr(mtx.output, decomposeMtx.inputMatrix)

    pm.connectAttr(decomposeMtx.outputRotate, source.rotate)

class Constraint(object):
    """ Base class for a constraint. """
    def __init__(self, *args, **kwargs):
        # Parse argument list.
        if len(args) >= 2:

            if isinstance(args[0], str):
                source = pm.ls(args[0])
                if len(source) < 1:
                    raise ValueError("More than one object matches name: {}".format(args[0]))
                self.source = source[0]

            elif isinstance(args[0], pm.nt.Transform):
                self.source = args[0]

            self.target = args[1]

        # Parse keywords and set attributes where appropriate
        for key in ('source', 'target', 'name'):
            if key in kwargs:
                setattr(self, key, kwargs[key])

        self._nodes = list()

class AimConstraint(Constraint):
    def __init__(self, *args, **kwargs):
        super(AimConstraint, self).__init__(*args, **kwargs)

    def _create_nodes(self):
        """ Create the required nodes for aim constraint set-up. """
