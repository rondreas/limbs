import pymel.core as pm
from limbs.core import ui

"""
    Example usages:
    >>> from ikfkSwitch import IKFKSwitch
    >>> import pymel.core as pm
    
    >>> # select hierarchy of ikChain, fkChain then bindChain
    >>> ikChain = pm.selected()
    
    >>> fkChain = pm.selected()
    
    >>> bindChain = pm.selected()
    
    >>> # select the controller object to add attributes onto.
    >>> ctrl = pm.selected()[0]
    >>> # create the switch
    >>> switch = IKFKSwitch(fkChain, ikChain, bindChain)
    >>> switch.attach(ctrl)
"""
class IKFKSwitch(object):
    """ Blend two sources, and output their result to target. """
    def __init__(self, sourceA, sourceB, target):
        self.inputs = [sourceA, sourceB]
        self.output = target

        self.blendNodes = []

        try:
            self.make_connections()
        except Exception as e:
            print e.message

    def make_connections(self):
        """ Create and set connections for each level in the hierarchy. """
        for sourceA, sourceB, target in zip(self.inputs[0], self.inputs[1], self.output):
            blendTranslate = pm.createNode('blendColors', name='tBlend{}'.format(target.nodeName()))
            blendRotate = pm.createNode('blendColors', name='rBlend{}'.format(target.nodeName()))

            pm.connectAttr(sourceA.translate, blendTranslate.color1)
            pm.connectAttr(sourceB.translate, blendTranslate.color2)

            pm.connectAttr(sourceA.rotate, blendRotate.color1)
            pm.connectAttr(sourceB.rotate, blendRotate.color2)

            pm.connectAttr(blendTranslate.output, target.translate)
            pm.connectAttr(blendRotate.output, target.rotate)

            self.blendNodes.append(blendTranslate)
            self.blendNodes.append(blendRotate)

    def attach(self, controller, name='IKFK'):
        """ Method for connecting all blend nodes blend attribute to a custom attribute on controller. """

        if isinstance(controller, str):
            controller = pm.ls(controller)[0]

        pm.addAttr(
            controller,
            longName=name,
            attributeType='double',
            keyable=True,
            max=1.0,    # sourceB
            min=0.0,    # sourceA
            defaultValue=0.0
        )

        for node in self.blendNodes:
            pm.connectAttr(controller.nodeName() + "." + name, node.blender)

    def focus(self):
        """ Show in Node Window. """
        # Get name of Node Editor Panels, 0th index should be the main node editor we're looking for.
        ne = pm.getPanel(scriptType='nodeEditorPanel')

        # Create a new tab, clear the tab, and add all associated nodes.
        pm.nodeEditor('{}NodeEditorEd'.format(ne[0]), q=True, getNodeList=True)


class IKFKSwitchGUI(ui.Window):
    pass

if __name__ == "__main__":
    pass
