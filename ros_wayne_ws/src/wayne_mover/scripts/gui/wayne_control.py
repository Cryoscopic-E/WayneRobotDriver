# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'wayne.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(558, 666)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopButton.setGeometry(QtCore.QRect(220, 560, 111, 51))
        self.stopButton.setStyleSheet("background-color:red")
        self.stopButton.setObjectName("stopButton")
        self.label_18 = QtWidgets.QLabel(self.centralwidget)
        self.label_18.setGeometry(QtCore.QRect(160, 150, 221, 31))
        self.label_18.setTextFormat(QtCore.Qt.AutoText)
        self.label_18.setScaledContents(True)
        self.label_18.setAlignment(QtCore.Qt.AlignCenter)
        self.label_18.setWordWrap(True)
        self.label_18.setObjectName("label_18")
        self.scrollArea = QtWidgets.QScrollArea(self.centralwidget)
        self.scrollArea.setGeometry(QtCore.QRect(110, 200, 331, 291))
        self.scrollArea.setWidgetResizable(True)
        self.scrollArea.setObjectName("scrollArea")
        self.scrollAreaWidgetContents = QtWidgets.QWidget()
        self.scrollAreaWidgetContents.setGeometry(QtCore.QRect(0, 0, 329, 289))
        self.scrollAreaWidgetContents.setObjectName("scrollAreaWidgetContents")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.scrollAreaWidgetContents)
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout.setSpacing(20)
        self.verticalLayout.setObjectName("verticalLayout")
        self.scrollArea.setWidget(self.scrollAreaWidgetContents)
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 10, 538, 118))
        self.layoutWidget.setObjectName("layoutWidget")
        self.gridLayout_valves = QtWidgets.QGridLayout(self.layoutWidget)
        self.gridLayout_valves.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_valves.setObjectName("gridLayout_valves")
        self.ValveLayout_0 = QtWidgets.QVBoxLayout()
        self.ValveLayout_0.setObjectName("ValveLayout_0")
        self.spinBox = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox.setMaximum(1)
        self.spinBox.setObjectName("spinBox")
        self.ValveLayout_0.addWidget(self.spinBox)
        self.label = QtWidgets.QLabel(self.layoutWidget)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.ValveLayout_0.addWidget(self.label)
        self.gridLayout_valves.addLayout(self.ValveLayout_0, 0, 0, 1, 1)
        self.ValveLayout_1 = QtWidgets.QVBoxLayout()
        self.ValveLayout_1.setObjectName("ValveLayout_1")
        self.spinBox_2 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_2.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_2.setMaximum(1)
        self.spinBox_2.setObjectName("spinBox_2")
        self.ValveLayout_1.addWidget(self.spinBox_2)
        self.label_2 = QtWidgets.QLabel(self.layoutWidget)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.ValveLayout_1.addWidget(self.label_2)
        self.gridLayout_valves.addLayout(self.ValveLayout_1, 0, 1, 1, 1)
        self.ValveLayout_2 = QtWidgets.QVBoxLayout()
        self.ValveLayout_2.setObjectName("ValveLayout_2")
        self.spinBox_3 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_3.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_3.setMaximum(1)
        self.spinBox_3.setObjectName("spinBox_3")
        self.ValveLayout_2.addWidget(self.spinBox_3)
        self.label_3 = QtWidgets.QLabel(self.layoutWidget)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.ValveLayout_2.addWidget(self.label_3)
        self.gridLayout_valves.addLayout(self.ValveLayout_2, 0, 2, 1, 1)
        self.ValveLayout_3 = QtWidgets.QVBoxLayout()
        self.ValveLayout_3.setObjectName("ValveLayout_3")
        self.spinBox_4 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_4.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_4.setMaximum(1)
        self.spinBox_4.setObjectName("spinBox_4")
        self.ValveLayout_3.addWidget(self.spinBox_4)
        self.label_4 = QtWidgets.QLabel(self.layoutWidget)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.ValveLayout_3.addWidget(self.label_4)
        self.gridLayout_valves.addLayout(self.ValveLayout_3, 0, 3, 1, 1)
        self.ValveLayout_4 = QtWidgets.QVBoxLayout()
        self.ValveLayout_4.setObjectName("ValveLayout_4")
        self.spinBox_5 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_5.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_5.setMaximum(1)
        self.spinBox_5.setObjectName("spinBox_5")
        self.ValveLayout_4.addWidget(self.spinBox_5)
        self.label_5 = QtWidgets.QLabel(self.layoutWidget)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.ValveLayout_4.addWidget(self.label_5)
        self.gridLayout_valves.addLayout(self.ValveLayout_4, 0, 4, 1, 1)
        self.ValveLayout_5 = QtWidgets.QVBoxLayout()
        self.ValveLayout_5.setObjectName("ValveLayout_5")
        self.spinBox_6 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_6.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_6.setMaximum(1)
        self.spinBox_6.setObjectName("spinBox_6")
        self.ValveLayout_5.addWidget(self.spinBox_6)
        self.label_6 = QtWidgets.QLabel(self.layoutWidget)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setObjectName("label_6")
        self.ValveLayout_5.addWidget(self.label_6)
        self.gridLayout_valves.addLayout(self.ValveLayout_5, 0, 5, 1, 1)
        self.ValveLayout_6 = QtWidgets.QVBoxLayout()
        self.ValveLayout_6.setObjectName("ValveLayout_6")
        self.spinBox_7 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_7.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_7.setMaximum(1)
        self.spinBox_7.setObjectName("spinBox_7")
        self.ValveLayout_6.addWidget(self.spinBox_7)
        self.label_7 = QtWidgets.QLabel(self.layoutWidget)
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.ValveLayout_6.addWidget(self.label_7)
        self.gridLayout_valves.addLayout(self.ValveLayout_6, 0, 6, 1, 1)
        self.ValveLayout_7 = QtWidgets.QVBoxLayout()
        self.ValveLayout_7.setObjectName("ValveLayout_7")
        self.spinBox_8 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_8.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_8.setMaximum(1)
        self.spinBox_8.setObjectName("spinBox_8")
        self.ValveLayout_7.addWidget(self.spinBox_8)
        self.label_8 = QtWidgets.QLabel(self.layoutWidget)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setObjectName("label_8")
        self.ValveLayout_7.addWidget(self.label_8)
        self.gridLayout_valves.addLayout(self.ValveLayout_7, 0, 7, 1, 1)
        self.ValveLayout_8 = QtWidgets.QVBoxLayout()
        self.ValveLayout_8.setObjectName("ValveLayout_8")
        self.spinBox_9 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_9.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_9.setMaximum(1)
        self.spinBox_9.setObjectName("spinBox_9")
        self.ValveLayout_8.addWidget(self.spinBox_9)
        self.label_9 = QtWidgets.QLabel(self.layoutWidget)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.ValveLayout_8.addWidget(self.label_9)
        self.gridLayout_valves.addLayout(self.ValveLayout_8, 1, 0, 1, 1)
        self.ValveLayout_9 = QtWidgets.QVBoxLayout()
        self.ValveLayout_9.setObjectName("ValveLayout_9")
        self.spinBox_10 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_10.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_10.setMaximum(1)
        self.spinBox_10.setObjectName("spinBox_10")
        self.ValveLayout_9.addWidget(self.spinBox_10)
        self.label_10 = QtWidgets.QLabel(self.layoutWidget)
        self.label_10.setAlignment(QtCore.Qt.AlignCenter)
        self.label_10.setObjectName("label_10")
        self.ValveLayout_9.addWidget(self.label_10)
        self.gridLayout_valves.addLayout(self.ValveLayout_9, 1, 1, 1, 1)
        self.ValveLayout_10 = QtWidgets.QVBoxLayout()
        self.ValveLayout_10.setObjectName("ValveLayout_10")
        self.spinBox_11 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_11.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_11.setMaximum(1)
        self.spinBox_11.setObjectName("spinBox_11")
        self.ValveLayout_10.addWidget(self.spinBox_11)
        self.label_11 = QtWidgets.QLabel(self.layoutWidget)
        self.label_11.setAlignment(QtCore.Qt.AlignCenter)
        self.label_11.setObjectName("label_11")
        self.ValveLayout_10.addWidget(self.label_11)
        self.gridLayout_valves.addLayout(self.ValveLayout_10, 1, 2, 1, 1)
        self.ValveLayout_11 = QtWidgets.QVBoxLayout()
        self.ValveLayout_11.setObjectName("ValveLayout_11")
        self.spinBox_12 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_12.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_12.setMaximum(1)
        self.spinBox_12.setObjectName("spinBox_12")
        self.ValveLayout_11.addWidget(self.spinBox_12)
        self.label_12 = QtWidgets.QLabel(self.layoutWidget)
        self.label_12.setAlignment(QtCore.Qt.AlignCenter)
        self.label_12.setObjectName("label_12")
        self.ValveLayout_11.addWidget(self.label_12)
        self.gridLayout_valves.addLayout(self.ValveLayout_11, 1, 3, 1, 1)
        self.ValveLayout_12 = QtWidgets.QVBoxLayout()
        self.ValveLayout_12.setObjectName("ValveLayout_12")
        self.spinBox_13 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_13.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_13.setMaximum(1)
        self.spinBox_13.setObjectName("spinBox_13")
        self.ValveLayout_12.addWidget(self.spinBox_13)
        self.label_13 = QtWidgets.QLabel(self.layoutWidget)
        self.label_13.setAlignment(QtCore.Qt.AlignCenter)
        self.label_13.setObjectName("label_13")
        self.ValveLayout_12.addWidget(self.label_13)
        self.gridLayout_valves.addLayout(self.ValveLayout_12, 1, 4, 1, 1)
        self.ValveLayout_13 = QtWidgets.QVBoxLayout()
        self.ValveLayout_13.setObjectName("ValveLayout_13")
        self.spinBox_14 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_14.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_14.setMaximum(1)
        self.spinBox_14.setObjectName("spinBox_14")
        self.ValveLayout_13.addWidget(self.spinBox_14)
        self.label_14 = QtWidgets.QLabel(self.layoutWidget)
        self.label_14.setAlignment(QtCore.Qt.AlignCenter)
        self.label_14.setObjectName("label_14")
        self.ValveLayout_13.addWidget(self.label_14)
        self.gridLayout_valves.addLayout(self.ValveLayout_13, 1, 5, 1, 1)
        self.ValveLayout_14 = QtWidgets.QVBoxLayout()
        self.ValveLayout_14.setObjectName("ValveLayout_14")
        self.spinBox_15 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_15.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_15.setMaximum(1)
        self.spinBox_15.setObjectName("spinBox_15")
        self.ValveLayout_14.addWidget(self.spinBox_15)
        self.label_15 = QtWidgets.QLabel(self.layoutWidget)
        self.label_15.setAlignment(QtCore.Qt.AlignCenter)
        self.label_15.setObjectName("label_15")
        self.ValveLayout_14.addWidget(self.label_15)
        self.gridLayout_valves.addLayout(self.ValveLayout_14, 1, 6, 1, 1)
        self.ValveLayout_15 = QtWidgets.QVBoxLayout()
        self.ValveLayout_15.setObjectName("ValveLayout_15")
        self.spinBox_16 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_16.setAlignment(QtCore.Qt.AlignCenter)
        self.spinBox_16.setMaximum(1)
        self.spinBox_16.setObjectName("spinBox_16")
        self.ValveLayout_15.addWidget(self.spinBox_16)
        self.label_16 = QtWidgets.QLabel(self.layoutWidget)
        self.label_16.setAlignment(QtCore.Qt.AlignCenter)
        self.label_16.setObjectName("label_16")
        self.ValveLayout_15.addWidget(self.label_16)
        self.gridLayout_valves.addLayout(self.ValveLayout_15, 1, 7, 1, 1)
        self.runOnceButton = QtWidgets.QPushButton(self.centralwidget)
        self.runOnceButton.setGeometry(QtCore.QRect(110, 500, 121, 41))
        self.runOnceButton.setObjectName("runOnceButton")
        self.runButton = QtWidgets.QPushButton(self.centralwidget)
        self.runButton.setGeometry(QtCore.QRect(320, 500, 121, 41))
        self.runButton.setObjectName("runButton")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Wayne Control SUPER MEGA GIGA"))
        self.stopButton.setText(_translate("MainWindow", "STOP"))
        self.label_18.setText(_translate("MainWindow", "SEQUENCES"))
        self.label.setText(_translate("MainWindow", "Valve 0"))
        self.label_2.setText(_translate("MainWindow", "Valve 1"))
        self.label_3.setText(_translate("MainWindow", "Valve 2"))
        self.label_4.setText(_translate("MainWindow", "Valve 3"))
        self.label_5.setText(_translate("MainWindow", "Valve 4"))
        self.label_6.setText(_translate("MainWindow", "Valve 5"))
        self.label_7.setText(_translate("MainWindow", "Valve 6"))
        self.label_8.setText(_translate("MainWindow", "Valve 7"))
        self.label_9.setText(_translate("MainWindow", "Valve 8"))
        self.label_10.setText(_translate("MainWindow", "Valve 9"))
        self.label_11.setText(_translate("MainWindow", "Valve 10"))
        self.label_12.setText(_translate("MainWindow", "Valve 11"))
        self.label_13.setText(_translate("MainWindow", "Valve 12"))
        self.label_14.setText(_translate("MainWindow", "Valve 13"))
        self.label_15.setText(_translate("MainWindow", "Valve 14"))
        self.label_16.setText(_translate("MainWindow", "Valve 15"))
        self.runOnceButton.setText(_translate("MainWindow", "Run Once"))
        self.runButton.setText(_translate("MainWindow", "Run"))
