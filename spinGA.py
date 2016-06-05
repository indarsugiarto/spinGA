#!/usr/bin/python

from PyQt4 import Qt
from qtGui import qtGui
import sys

def main():
    """
    The main program starts here
    """
    app = Qt.QApplication(sys.argv)
    gui = qtGui()
    gui.show()
    sys.exit(app.exec_())

if __name__=="__main__":
    main()
