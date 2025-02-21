import rclpy  # Python library for ROS 2
import cv2  # OpenCV library
from PyQt5.QtGui import QPixmap
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import pyqtSignal, pyqtSlot, Qt, QThread
import time
from PyQt5.QtWidgets import QMainWindow, QApplication, QMenu, QAction, QStyle, qApp, QMessageBox
from PyQt5.QtCore import Qt, QTimer
import numpy as np
from std_msgs.msg import Int32MultiArray

import sys


class Ui_MainWindow(object):

    def __init__(self):
        self.float_topic_name = "camera"
        self.timer = QTimer()
        self.timer.timeout.connect(self.timer_image_update)
        self.disply_width = 640
        self.display_height = 480
        self.br = CvBridge()
        self.frame = np.zeros((self.display_height, self.disply_width)).astype(np.uint8)
        self.got_frame = False

        # Color to calibrate (selected with radio button)
        self.calibrate_color = "Brown"
        self.color_number = 1

        #     Storing thresholds values from sliders
        #     red color
        self.R_lower = 0
        self.R_upper = 0
        #     blue color
        self.B_lower = 0
        self.B_upper = 0
        #    green color
        self.G_lower = 0
        self.G_upper = 0

        # value to handle stoping and playing video
        self.run_video = True
        

    def ros_shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def ros_init(self):
        print("ros_init")
        rclpy.init(args=None)
        self.node = Node('detector_gui')
        self.sub = self.node.create_subscription(
            Image,
            self.float_topic_name,
            self.image_callback,
            10,
        )
        self.thresholds_publisher = self.node.create_publisher(Int32MultiArray,
                                                               "detector_thresholds",
                                                               10)
        # spin once, timeout_sec 5[s]
        timeout_sec_rclpy = 5
        timeout_init = time.time()
        rclpy.spin_once(self.node, timeout_sec=timeout_sec_rclpy)
        timeout_end = time.time()
        ros_connect_time = timeout_end - timeout_init
        if ros_connect_time >= timeout_sec_rclpy:
            print("ros connection successful")
        else:
            print("ros connection failed")
        #     Start timer
        # 10 FPS
        self.timer.start(100)

    def timer_image_update(self):
        # print("timer update")
        if self.run_video:
            rclpy.spin_once(self.node, timeout_sec=1)
            if self.got_frame:
                converted_frame = self.convert_cv_qt(self.frame)
                # Display normal image on label
                self.label_2.setPixmap(converted_frame)
        # Display thresholded image on label2
        # self.frame = self.frame.astype(np.uint8)
        mask = cv2.inRange(self.frame, np.array([self.R_lower, self.G_lower, self.B_lower]),
                           np.array([self.R_upper, self.G_upper, self.B_upper]))
        masked_frame = cv2.bitwise_and(self.frame, self.frame, mask=mask)
        self.label.setPixmap(self.convert_cv_qt(masked_frame))

        # print("timer update end")

        # self.timer.start(1000)

    def ros_publish_thresholds(self):
        # Publishing thresholds from gui
        # Thresholds are published in int32 array in format:
        # [color_number, R_lower, G_lower, B_lower, R_upper, G_upper, B_upper]
        # color_number: 1-Brown, 2-Beige, 3-Golden
        thres_msg = Int32MultiArray()
        thres_msg.data = [self.color_number, self.R_lower, self.G_lower, self.B_lower, self.R_upper, self.G_upper,
                          self.B_upper]
        self.thresholds_publisher.publish(thres_msg)
        print("thresholds published")

    def image_callback(self, img):
        print("image callback")
        frame = self.br.imgmsg_to_cv2(img)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        self.got_frame = True
        self.frame = frame.astype(np.uint8)

    def convert_cv_qt(self, cv_img):
        """Convert from an opencv image to QPixmap"""
        # rgb_image = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        rgb_image = cv_img
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        convert_to_Qt_format = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_BGR888)
        p = convert_to_Qt_format.scaled(self.disply_width, self.display_height, Qt.KeepAspectRatio)
        return QPixmap.fromImage(p)

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1454, 894)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.layoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.layoutWidget.setGeometry(QtCore.QRect(10, 10, 1431, 831))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setSizeConstraint(QtWidgets.QLayout.SetNoConstraint)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtWidgets.QLabel(self.layoutWidget)
        self.label_2.setMinimumSize(QtCore.QSize(640, 480))
        self.label_2.setMaximumSize(QtCore.QSize(640, 480))
        self.label_2.setLineWidth(1)
        self.label_2.setText("")
        self.label_2.setPixmap(QtGui.QPixmap())
        self.label_2.setScaledContents(False)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.label = QtWidgets.QLabel(self.layoutWidget)
        self.label.setMinimumSize(QtCore.QSize(640, 480))
        self.label.setMaximumSize(QtCore.QSize(640, 480))
        self.label.setText("")
        self.label.setPixmap(QtGui.QPixmap())
        self.label.setScaledContents(False)
        self.label.setObjectName("label")
        self.horizontalLayout.addWidget(self.label)
        self.verticalLayout_4.addLayout(self.horizontalLayout)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_3 = QtWidgets.QLabel(self.layoutWidget)
        self.label_3.setObjectName("label_3")
        self.horizontalLayout_2.addWidget(self.label_3)
        self.horizontalSlider = QtWidgets.QSlider(self.layoutWidget)
        self.horizontalSlider.setMaximum(255)
        self.horizontalSlider.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider.setObjectName("horizontalSlider")
        self.horizontalLayout_2.addWidget(self.horizontalSlider)
        self.spinBox = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox.setMaximum(255)
        self.spinBox.setObjectName("spinBox")
        self.horizontalLayout_2.addWidget(self.spinBox)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_4 = QtWidgets.QLabel(self.layoutWidget)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_3.addWidget(self.label_4)
        self.horizontalSlider_2 = QtWidgets.QSlider(self.layoutWidget)
        self.horizontalSlider_2.setMaximum(255)
        self.horizontalSlider_2.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_2.setObjectName("horizontalSlider_2")
        self.horizontalLayout_3.addWidget(self.horizontalSlider_2)
        self.spinBox_2 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_2.setMaximum(255)
        self.spinBox_2.setObjectName("spinBox_2")
        self.horizontalLayout_3.addWidget(self.spinBox_2)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_5 = QtWidgets.QLabel(self.layoutWidget)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_4.addWidget(self.label_5)
        self.horizontalSlider_3 = QtWidgets.QSlider(self.layoutWidget)
        self.horizontalSlider_3.setMaximum(255)
        self.horizontalSlider_3.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_3.setObjectName("horizontalSlider_3")
        self.horizontalLayout_4.addWidget(self.horizontalSlider_3)
        self.spinBox_3 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_3.setMaximum(255)
        self.spinBox_3.setObjectName("spinBox_3")
        self.horizontalLayout_4.addWidget(self.spinBox_3)
        self.verticalLayout.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_8.addLayout(self.verticalLayout)
        self.verticalLayout_2 = QtWidgets.QVBoxLayout()
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.horizontalSlider_4 = QtWidgets.QSlider(self.layoutWidget)
        self.horizontalSlider_4.setMaximum(255)
        self.horizontalSlider_4.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_4.setObjectName("horizontalSlider_4")
        self.horizontalLayout_5.addWidget(self.horizontalSlider_4)
        self.spinBox_4 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_4.setMaximum(255)
        self.spinBox_4.setObjectName("spinBox_4")
        self.horizontalLayout_5.addWidget(self.spinBox_4)
        self.verticalLayout_2.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.horizontalSlider_5 = QtWidgets.QSlider(self.layoutWidget)
        self.horizontalSlider_5.setMaximum(255)
        self.horizontalSlider_5.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_5.setObjectName("horizontalSlider_5")
        self.horizontalLayout_6.addWidget(self.horizontalSlider_5)
        self.spinBox_5 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_5.setMaximum(255)
        self.spinBox_5.setObjectName("spinBox_5")
        self.horizontalLayout_6.addWidget(self.spinBox_5)
        self.verticalLayout_2.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.horizontalSlider_6 = QtWidgets.QSlider(self.layoutWidget)
        self.horizontalSlider_6.setMaximum(255)
        self.horizontalSlider_6.setOrientation(QtCore.Qt.Horizontal)
        self.horizontalSlider_6.setObjectName("horizontalSlider_6")
        self.horizontalLayout_7.addWidget(self.horizontalSlider_6)
        self.spinBox_6 = QtWidgets.QSpinBox(self.layoutWidget)
        self.spinBox_6.setMaximum(255)
        self.spinBox_6.setObjectName("spinBox_6")
        self.horizontalLayout_7.addWidget(self.spinBox_6)
        self.verticalLayout_2.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_8.addLayout(self.verticalLayout_2)
        self.horizontalLayout_9.addLayout(self.horizontalLayout_8)
        self.verticalLayout_3 = QtWidgets.QVBoxLayout()
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.radioButton = QtWidgets.QRadioButton(self.layoutWidget)
        self.radioButton.setObjectName("radioButton")
        self.verticalLayout_3.addWidget(self.radioButton)
        self.radioButton_3 = QtWidgets.QRadioButton(self.layoutWidget)
        self.radioButton_3.setObjectName("radioButton_3")
        self.verticalLayout_3.addWidget(self.radioButton_3)
        self.radioButton_2 = QtWidgets.QRadioButton(self.layoutWidget)
        self.radioButton_2.setObjectName("radioButton_2")
        self.verticalLayout_3.addWidget(self.radioButton_2)
        self.pushButton = QtWidgets.QPushButton(self.layoutWidget)
        self.pushButton.setObjectName("Stop/Start")
        self.verticalLayout_3.addWidget(self.pushButton)
        self.pushButton_2 = QtWidgets.QPushButton(self.layoutWidget)
        self.pushButton_2.setObjectName("OK")
        self.verticalLayout_3.addWidget(self.pushButton_2)
        self.horizontalLayout_9.addLayout(self.verticalLayout_3)
        self.verticalLayout_4.addLayout(self.horizontalLayout_9)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtWidgets.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 1454, 23))
        self.menubar.setObjectName("menubar")
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)
        # connect functions with sliders and buttons
        self.connect_signals()
        # Set upper sliders to max value (255)
        self.set_sliders_default()
        # initialize ros connection and start subscriber
        self.ros_init()

    def connect_signals(self):
        # Sliders
        self.horizontalSlider.valueChanged.connect(self.slider_1_changed)
        self.horizontalSlider_2.valueChanged.connect(self.slider_2_changed)
        self.horizontalSlider_3.valueChanged.connect(self.slider_3_changed)
        self.horizontalSlider_4.valueChanged.connect(self.slider_4_changed)
        self.horizontalSlider_5.valueChanged.connect(self.slider_5_changed)
        self.horizontalSlider_6.valueChanged.connect(self.slider_6_changed)

        # Radio buttons
        self.radioButton.toggled.connect(self.radio_button_update)
        self.radioButton.setChecked(True)
        self.radioButton_2.toggled.connect(self.radio_button_update)
        self.radioButton_3.toggled.connect(self.radio_button_update)

        # Ok button
        self.pushButton_2.clicked.connect(self.ok_button_clicked)
        # Stop/Start buttom
        self.pushButton.clicked.connect(self.stop_start_button_clicked)

        # Spin Boxes
        self.spinBox.valueChanged.connect(self.horizontalSlider.setValue)
        self.spinBox_2.valueChanged.connect(self.horizontalSlider_2.setValue)
        self.spinBox_3.valueChanged.connect(self.horizontalSlider_3.setValue)
        self.spinBox_4.valueChanged.connect(self.horizontalSlider_4.setValue)
        self.spinBox_5.valueChanged.connect(self.horizontalSlider_5.setValue)
        self.spinBox_6.valueChanged.connect(self.horizontalSlider_6.setValue)

    def set_sliders_default(self):
        self.horizontalSlider.setValue(0)
        self.horizontalSlider_2.setValue(0)
        self.horizontalSlider_3.setValue(0)
        self.horizontalSlider_4.setValue(255)
        self.horizontalSlider_5.setValue(255)
        self.horizontalSlider_6.setValue(255)

    def slider_1_changed(self):
        # Lower RED
        value = self.horizontalSlider.value()
        self.R_lower = value

        # Set spinbox to slider value
        self.spinBox.setValue(value)

    def slider_2_changed(self):
        # Lower GREEN
        value = self.horizontalSlider_2.value()
        self.G_lower = value

        # Set spinbox to slider value
        self.spinBox_2.setValue(value)

    def slider_3_changed(self):
        #  Lower BLUE
        value = self.horizontalSlider_3.value()
        self.B_lower = value

        # Set spinbox to slider value
        self.spinBox_3.setValue(value)


    def slider_4_changed(self):
        # Upper RED
        value = self.horizontalSlider_4.value()
        self.R_upper = value

        # Set spinbox to slider value
        self.spinBox_4.setValue(value)


    def slider_5_changed(self):
        #  Upper GREEN
        value = self.horizontalSlider_5.value()
        self.G_upper = value

        # Set spinbox to slider value
        self.spinBox_5.setValue(value)


    def slider_6_changed(self):
        #  Upper BLUE
        value = self.horizontalSlider_6.value()
        self.B_upper = value

        # Set spinbox to slider value
        self.spinBox_6.setValue(value)


    def radio_button_update(self):
        if self.radioButton.isChecked():
            self.calibrate_color = "Brown"
            self.color_number = 1
        if self.radioButton_2.isChecked():
            self.calibrate_color = "Golden"
            self.color_number = 3
        if self.radioButton_3.isChecked():
            self.color_number = 2
            self.calibrate_color = "Beige"
        print(self.calibrate_color)

    def ok_button_clicked(self):
        self.ros_publish_thresholds()
        self.show_popup()
        self.set_sliders_default()

    def stop_start_button_clicked(self):
        self.run_video = not self.run_video

    def show_popup(self):
        msg = QMessageBox()
        msg.setWindowTitle("Thresholds calibration")
        msg.setText("Thresholds calibrated for color " + self.calibrate_color)
        msg.setIcon(QMessageBox.Information)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.setDefaultButton(QMessageBox.Ok)
        # msg.setInformativeText("informative text, ya!")

        msg.setDetailedText("details")

        # msg.buttonClicked.connect(self.popup_button)
        x = msg.exec_()

    def popup_button(self, i):
        print(i.text())

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.label_3.setText(_translate("MainWindow", "Red"))
        self.label_4.setText(_translate("MainWindow", "Green"))
        self.label_5.setText(_translate("MainWindow", "Blue"))
        self.radioButton.setText(_translate("MainWindow", "Brown"))
        self.radioButton_3.setText(_translate("MainWindow", "Beige"))
        self.radioButton_2.setText(_translate("MainWindow", "Golden"))
        self.pushButton.setText(_translate("MainWindow", "Stop/Start"))
        self.pushButton_2.setText(_translate("MainWindow", "OK"))


def main(args=None):
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
