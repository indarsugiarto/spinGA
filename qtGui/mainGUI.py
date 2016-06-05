# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'mainGUI.ui'
#
# Created by: PyQt4 UI code generator 4.11.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_pySpinGA(object):
    def setupUi(self, pySpinGA):
        pySpinGA.setObjectName(_fromUtf8("pySpinGA"))
        pySpinGA.resize(319, 100)
        self.pbGo = QtGui.QPushButton(pySpinGA)
        self.pbGo.setGeometry(QtCore.QRect(10, 10, 101, 29))
        self.pbGo.setObjectName(_fromUtf8("pbGo"))

        self.retranslateUi(pySpinGA)
        QtCore.QMetaObject.connectSlotsByName(pySpinGA)

    def retranslateUi(self, pySpinGA):
        pySpinGA.setWindowTitle(_translate("pySpinGA", "GA on SpiNNaker", None))
        self.pbGo.setText(_translate("pySpinGA", "Go", None))

