from PyQt4 import QtGui
import mainGUI # from mainGUI.py, which originates from mainGUI.ui

nPop = 4    # if we run in Spin3, it can have up to 4 populations

from myObjValPlotter import objPlotter

class qtGui(QtGui.QWidget, mainGUI.Ui_pySpinGA):
#class qtGui(mainGUI.Ui_pySpinGA):
    def __init__(self, parent=None):
        super(qtGui, self).__init__(parent)
        self.setupUi(self)
        self.objPlot = objPlotter(nPop)


