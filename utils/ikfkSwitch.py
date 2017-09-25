import pymel.core as pymel


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
        ''' Create and set connections for each level in the hierarchy. '''

        for level in zip(self.inputs[0], self.inputs[1], self.output):
            blendTranslate = pymel.createNode('blendColors', name='blendTranslate')
            blendRotate = pymel.createNode('blendColors', name='blendRotate')

            pymel.connectAttr(level[0].translate, blendTranslate.color1)
            pymel.connectAttr(level[1].translate, blendTranslate.color2)

            pymel.connectAttr(level[0].rotate, blendRotate.color1)
            pymel.connectAttr(level[1].rotate, blendRotate.color2)

            pymel.connectAttr(blendTranslate.output, level[2].translate)
            pymel.connectAttr(blendRotate.output, level[2].rotate)

            self.blendNodes.append(blendTranslate)
            self.blendNodes.append(blendRotate)

    def attach(self, controller, name='IKFK'):
        ''' Method for connecting all blend nodes blend attribute to a custom attribute on controller. '''
        pymel.addAttr(
            controller,
            longName=name,
            attributeType='double',
            keyable=True,
            max=1.0,
            min=0.0,
            defaultValue=0.0
        )

        for node in self.blendNodes:
            pymel.connectAttr(controller.name() + "." + name, node.blender)

    def focus(self):
        """ Show in Node Window. """
        pass


if __name__ == "__main__":
    pass
