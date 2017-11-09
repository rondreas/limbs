import pymel.core as pm


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

def remove_parent_constraint():
    pass