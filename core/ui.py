from maya import OpenMayaUI as omui
import os

try:
    from PySide2.QtCore import *
    from PySide2.QtGui import *
    from PySide2.QtWidgets import *
    from PySide2 import __version__
    from shiboken2 import wrapInstance
except ImportError:
    from PySide.QtCore import *
    from PySide.QtGui import *
    from PySide import __version__
    from shiboken import wrapInstance

mayaMainWindowPtr = omui.MQtUtil.mainWindow()
mayaMainWindow = wrapInstance(long(mayaMainWindowPtr), QWidget)

class Window(QWidget):
    def __init__(self, parent=mayaMainWindow):
        super(Window, self).__init__(parent=parent)

        # Think it was Chad Vernon who had a video on Qt for maya saying there was a special case for OSX users.
        if os.name is 'posix':
            self.setWindowFlags(Qt.Tool)
        else:
            self.setWindowFlags(Qt.Window)

if __name__ == "__main__":
    pass
