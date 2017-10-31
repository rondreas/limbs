import pymel.core as pm

def parent(source, target, maintainOffset=True):
    """ Creates a parent constrain using maya nodes.
        Thanks Cult of Rig and http://bindpose.com. """

    # Create required nodes,
    multMtx = pm.createNode('multMatrix', name='multMtx')
    decompMtx = pm.createNode('decomposeMatrix', name='decompMtx')

    if maintainOffset:
        offsetMtx = pm.createNode('multMatrix', name='offsetMtx')
        pm.connectAttr(target.worldMatrix[0], offsetMtx.matrixIn[0])
        pm.connectAttr(source.worldInverseMatrix[0], offsetMtx.matrixIn[1])
        pm.setAttr(multMtx.matrixIn[0], offsetMtx.getAttr('matrixSum'))

    # Assign connections
    pm.connectAttr(source.worldMatrix[0], multMtx.matrixIn[1])
    pm.connectAttr(target.parentInverseMatrix[0], multMtx.matrixIn[2])

    pm.connectAttr(multMtx.matrixSum, decompMtx.inputMatrix)

    pm.connectAttr(decompMtx.outputTranslate, target.translate)
    pm.connectAttr(decompMtx.outputRotate, target.rotate)
    pm.connectAttr(decompMtx.outputScale, target.scale)