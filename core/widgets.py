from limbs.core import ui

class Spoiler(ui.QWidget):
    """ """
    def __init__(self, title):
        super(Spoiler, self).__init__()

        mainLayout = ui.QGridLayout()
        self.toggleButton = ui.QToolButton()
        headerLine = ui.QFrame()
        self.toggleAnimation = ui.QParallelAnimationGroup()
        self.contentArea = ui.QScrollArea()
        self.animationDuration = 300

        self.toggleButton.setStyleSheet("QToolButton { border: none; }")
        self.toggleButton.setToolButtonStyle(ui.Qt.ToolButtonTextBesideIcon)
        self.toggleButton.setArrowType(ui.Qt.RightArrow)
        self.toggleButton.setText(title)
        self.toggleButton.setCheckable(True)
        self.toggleButton.setChecked(False)

        headerLine.setFrameShape(ui.QFrame.HLine)
        headerLine.setFrameShadow(ui.QFrame.Sunken)
        headerLine.setSizePolicy(ui.QSizePolicy.Expanding, ui.QSizePolicy.Maximum)

        self.contentArea.setStyleSheet("QScrollArea { background-color: white; border: none; }")
        self.contentArea.setSizePolicy(ui.QSizePolicy.Expanding, ui.QSizePolicy.Fixed)
        # start out collapsed
        self.contentArea.setMaximumHeight(0)
        self.contentArea.setMinimumHeight(0)
        # let the entire widget grow and shrink with its content
        self.toggleAnimation.addAnimation(ui.QPropertyAnimation(self.toggleAnimation, "minimumHeight"))
        self.toggleAnimation.addAnimation(ui.QPropertyAnimation(self.toggleAnimation, "maximumHeight"))
        self.toggleAnimation.addAnimation(ui.QPropertyAnimation(self.contentArea, "maximumHeight"))
        # don't waste space
        mainLayout.setVerticalSpacing(0)
        mainLayout.setContentsMargins(0, 0, 0, 0)
        mainLayout.addWidget(self.toggleButton, 0, 0, 1, 1, ui.Qt.AlignLeft)
        mainLayout.addWidget(headerLine, 0, 2, 1, 1)
        mainLayout.addWidget(self.contentArea, 1, 0, 1, 3)
        self.setLayout(mainLayout)

        self.toggleButton.clicked.connect(self.toggleClicked)

    def setContentLayout(self, layout):
        """ Need to be called whenever the layout changes. """
        self.contentArea.setLayout(layout)

        collapsedHeight = self.sizeHint().height() - self.contentArea.maximumHeight()
        contentHeight = layout.sizeHint().height()

        for i in range(self.toggleAnimation.animationCount() - 1):
            spoilerAnimation = self.toggleAnimation.animationAt(i)
            spoilerAnimation.setDuration(self.animationDuration)
            spoilerAnimation.setStartValue(collapsedHeight)
            spoilerAnimation.setEndValue(collapsedHeight + contentHeight)

        contentAnimation = self.toggleAnimation.animationAt(self.toggleAnimation.animationCount() - 1)
        contentAnimation.setDuration(self.animationDuration)
        contentAnimation.setStartValue(0)
        contentAnimation.setEndValue(contentHeight)

    def toggleClicked(self):

        self.toggleButton.setArrowType(
            ui.Qt.ArrowType.DownArrow if self.toggleButton.isChecked() else ui.Qt.ArrowType.RightArrow
        )

        self.toggleAnimation.setDirection(
            ui.QAbstractAnimation.Forward if self.toggleButton.isChecked() else ui.QAbstractAnimation.Backward
        )

        self.toggleAnimation.start()

class CollapsibleGroupBox(ui.QGroupBox):
    def __init__(self):
        super(CollapsibleGroupBox, self).__init__()

        self.createLayout()

    def createLayout(self):
        """ """
        self.titleWidget = ui.QWidget()
        self.contentWidget = ui.QWidget()

        layout = ui.QVBoxLayout()
        self.setLayout(layout)

        titleLayout = ui.QHBoxLayout()
        collapseButton = CollapseButton()
        collapseButton.clicked.connect(self.toggleContents)
        titleLayout.addWidget(collapseButton)
        self.titleWidget.setLayout(titleLayout)

        # Add layouts and widgets
        layout.addWidget(self.titleWidget)
        layout.addWidget(self.contentWidget)

    def toggleContents(self):
        """ Show/Hide the widget containing the contents. """
        self.contentWidget.setVisible(not self.contentWidget.isVisible())

class CollapseButton(ui.QPushButton):
    """ """
    def __init__(self):
        super(CollapseButton, self).__init__()