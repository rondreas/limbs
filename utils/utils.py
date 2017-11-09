import pymel.core as pm

def get_nodes():
    # Get name of Node Editor Panels,
    ne = pm.getPanel(scriptType='nodeEditorPanel')

    # Get List of Nodes in the main Node Editor Panel.
    nodeList = pm.nodeEditor('{}NodeEditorEd'.format(ne[0]), q=True, getNodeList=True)

    # For each node in Nodes, get connections and include only connections to and from nodes in the node list.
    connections = list()
    for node in nodeList:
        connections.append([attr for attr in pm.listConnections(node, d=False, s=True, c=True, p=True) if attr.nodeName() in nodeList])

    for node in nodeList:
        # List of tuple pairs of attributes connected
        connections = pm.listConnections(node, d=False, s=True, c=True, p=True)
        for pair in connections:
            nodes = []

    return connections

def swap_inputs(node):
    """ Get inputs for node, and swap inputs1 with inputs2 for selected or specified node """

    # TODO save as a hotkey and make context aware so this only runs if nodeEditor is active window.

    # List all incoming connections to node with attribute names.
    inputs = pm.listConnections(node, d=False, s=True, c=True, p=True)

    # Disconnect attributes
    pm.disconnectAttr(inputs[0][1], inputs[0][0])
    pm.disconnectAttr(inputs[1][1], inputs[1][0])

    # Connect attributes
    pm.connectAttr(inputs[0][1], inputs[1][0])
    pm.connectAttr(inputs[1][1], inputs[0][0])

def replace_constraint_target(former, current):
    """ Check former for constraints where it is a target, and replace connections to instead point toward current."""
    pass