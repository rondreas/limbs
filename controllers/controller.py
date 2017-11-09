import pymel.core as pm
from limbs.utils import constraints


def zero_transform(transform):
    """ Zero out translate, rotate and set scale to 1. """
    transform.setAttr('translate', (0, 0, 0))
    transform.setAttr('rotate', (0, 0, 0))
    transform.setAttr('scale', (1, 1, 1))

def match_transform(driver=None, driven=None):
    # TODO catch exceptions, only handle kTransforms
    if not driver and not driven:
        driver, driven = pm.selected()

    pm.xform(
        driven,
        ws=True,
        translation=pm.xform(driver, q=True, ws=True, t=True),
        rotation=pm.xform(driver, q=True, ws=True, ro=True),
        scale=pm.xform(driver, q=True, ws=True, s=True),
    )

def mirror(transform):
    grp = pm.group(transform, name='tmpMirrorGrp', relative=True)
    grp.setAttr('scaleX', -1)
    pm.parent(transform, world=True)
    pm.delete(grp)

def srt_buffer(target=None, parent=None, child=None):
    """ Create a null transform at selected or target. """
    if not target:
        target=pm.selected()[0]

    srtBuffer = pm.group(
        name=target.nodeName()+"_srtBuffer",
        empty=True,
    )

    match_transform(srtBuffer, target)

    if child:
        pm.parent(child, srtBuffer)

    if parent:
        pm.parent(srtBuffer, parent)

    return srtBuffer

def connect_hierarchies():
    """ With root of two hierarchies selected, iterate over them and connect the translates and rotates of first
    onto second. """
    driver, driven = pm.selected()
    # Connect translates and rotates from hierarchy driver to driven, select
    driverChain = driver.getChildren(allDescendents=True, type="transform")
    drivenChain = driven.getChildren(allDescendents=True, type="transform")
    for driver, driven in zip(driverChain, drivenChain):
        driver.t >> driven.t
        driver.r >> driven.r

def fk_chain(transform):
    """ Create forward kinematic links on list of transforms. parent constrain obj->nextObj.parent"""
    # TODO, Don't add FK_ prefix if one is already present.
    # TODO, verify that all given transforms have a parent
    for x in range(len(transform) - 1):
        constraints.parent(
            transform[x],
            transform[x + 1].listRelatives(parent=True)[0],
            name='FK_' + transform[x].nodeName(),
            maintainOffset=True,
        )

if __name__ == '__main__':
    pass
