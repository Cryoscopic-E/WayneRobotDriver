from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication
from functools import partial
import sys
import wayne_control as gui

class WayneControlApp(QtWidgets.QMainWindow, gui.Ui_MainWindow):
    def __init__(self, parent=None):
        super(WayneControlApp, self).__init__(parent)
        self.setupUi(self)
        self.spin_boxes = {
            '0':self.spinBox,
            '1':self.spinBox_2,
            '2':self.spinBox_3,
            '3':self.spinBox_4,
            '4':self.spinBox_5,
            '5':self.spinBox_6,
            '6':self.spinBox_7,
            '7':self.spinBox_8,
            '8':self.spinBox_9,
            '9':self.spinBox_10,
            '10':self.spinBox_11,
            '11':self.spinBox_12,
            '12':self.spinBox_13,
            '13':self.spinBox_14,
            '14':self.spinBox_15,
            '15':self.spinBox_16,
        }
    
        for index, spinbox in self.spin_boxes.items():
            spinbox.valueChanged.connect(partial(self.handle_spinbox_change, index, spinbox))


        self.runOnceButton.clicked.connect(self.handle_run_once)
        self.runButton.pressed.connect(self.handle_run_start)
        self.runButton.released.connect(self.handle_run_stop)
        self.stopButton.clicked.connect(self.stop)

    def handle_spinbox_change(self,index, spinbox):
        if spinbox.value() == 1:
            spinbox.setStyleSheet('background-color:red')
        else:
            spinbox.setStyleSheet('background-color:white')

    def handle_run_once(self):
        print("Start sequence once")

    def handle_run_start(self):
        print("Start sequence")

    def handle_run_stop(self):
        print("Stop sequence")

    def stop(self):
        print("DEFLATE & RESET")

def main():
    app = QApplication(sys.argv)
    form = WayneControlApp()
    form.show()
    app.exec_()

if __name__ == '__main__':
    main()