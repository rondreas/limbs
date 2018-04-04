"""
    Editing Maya UI with python Qt,
"""

import pymel.core as pm
from limbs.core import ui

# Get the node editor as a Qt object,
ne = pm.getPanel(scriptType='nodeEditorPanel')[0]
ne_qt = ne.asQtObject()

# Adding a custom menu to the menubar
menubar = ne_qt.children()[1]

my_menu = ui.QMenu("Hello")

menubar.addMenu(my_menu)