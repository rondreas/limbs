import pymel.core as pm
from limbs.utils import constraints

def ctrl(target, offset=True):
    """ Create the most basic of controllers at the target, or selected object."""
    circle = pm.circle(
        name=target.nodeName() + '_CTRL',
        normal=(1, 0, 0)
    )[0]

    if offset:
        srt_buffer(target=target, child=circle)
        zero_transform(circle)
    else:
        pm.parent(circle, target, relative=True)
        pm.parent(circle, world=True)

    return circle

def replace_shape(old=None, new=None, maintainOffset=False):
    """ Replace the shape of old with new shape."""

    # If nothing specified use selection,
    if not old and not new:
        # Not following the Maya standard of 'driver driven' but using the python str replace() order instead.
        old, new = pm.selected()

    # Get the shape we want to use instead of old
    shape = new.getShape()

    # Remove the shape of old,
    pm.delete(old.getShape())

    # Parent the new shape under the old transform
    pm.parent(
        shape,
        old,
        shape=True,
        relative=False if maintainOffset else True,
    )

    # Set shape name according to Maya standards,
    pm.rename(shape, old.nodeName() + 'Shape')

    # Remove transform of the new object, this will otherwise be empty and clutter the scene.
    pm.delete(new)

def zero_transform(transform):
    """ Zero out translate, rotate and set scale to 1. """
    transform.setAttr('translate', (0, 0, 0))
    transform.setAttr('rotate', (0, 0, 0))
    transform.setAttr('scale', (1, 1, 1))

def match_transform(driver=None, driven=None):
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

    match_transform(target, srtBuffer)

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
    # TODO, verify that all given transforms have a parent offset group/ srt buffer
    for x in range(len(transform) - 1):
        name = 'FK_' + transform[x].nodeName() if not transform[x].nodeName().startswith('FK_') else transform[x].nodeName()
        constraints.parent(
            transform[x],
            transform[x + 1].listRelatives(parent=True)[0],
            name=name,
            maintainOffset=True,
        )

if __name__ == '__main__':
    pass
