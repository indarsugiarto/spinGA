from PyQt4 import QtGui
import mainGUI # from mainGUI.py, which originates from mainGUI.ui

class qtGui(QtGui.QWidget, mainGUI.Ui_pySpinGA):
#class qtGui(mainGUI.Ui_pySpinGA):
    def __init__(self, parent=None):
        super(qtGui, self).__init__(parent)
        self.setupUi(self)

