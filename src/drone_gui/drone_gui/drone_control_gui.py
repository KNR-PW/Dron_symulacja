import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import sys
import time
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QMessageBox, qApp
from rcl_interfaces.msg import Log
from PyQt5.QtCore import Qt, QTimer, QObject, QThread, pyqtSignal
from drone_interfaces.action import GotoRelative, Takeoff, Arm
from drone_interfaces.srv import SetYaw, GetAttitude, GetLocationRelative, SetMode, SetServo, TakePhoto


class BoxLogger(QObject):
    def __init__(self, textEdit, node):
        super().__init__()
        self.log_info = " LOGGER INIT LOG"
        self.should_run = True
        self.new_log = False
        self.text_edit = textEdit
        self.node = node

    def run(self):
        self.text_edit.append(self.log_info)
        rclpy.spin(self.node)

    def add_log(self, log):
        self.log_info = log
        self.new_log = True

    def log_callback(self, cb):
        print('dupalog')


class Ui_MainWindow(object):
    def __init__(self):
        self.ros_timer = QTimer()
        self.gps_timer = QTimer()
        self.log_signal = pyqtSignal()
        self.log_thread = QThread()

        self.ros_timer.timeout.connect(self.timer_ros_update)
        self.gps_timer.timeout.connect(self.gps_timer_update)
        self.step = 0.0

    def init_box_logger(self):
        self.box_logger = BoxLogger(self.log_textEdit, self.node)
        self.log_thread = QThread()
        self.box_logger.moveToThread(self.log_thread)
        self.log_thread.started.connect(self.box_logger.run)
        self.log_thread.start()

    def ros_init(self):
        rclpy.init(args=None)
        self.node = Node('drone_control_gui')
        self.ros_init_clients()
        self.log_subscription = self.node.create_subscription(
            Log,
            '/rosout',
            self.log_callback,
            10)
        self.ros_timer.start(100)
        self.gps_timer.start(1000)

    def ros_init_clients(self):
        ## DECLARE SERVICES
        self.attitude_cli = self.node.create_client(GetAttitude, 'get_attitude')
        self.gps_cli = self.node.create_client(GetLocationRelative, 'get_location_relative')
        self.servo_cli = self.node.create_client(SetServo, 'set_servo')
        self.yaw_cli = self.node.create_client(SetYaw, 'set_yaw')
        self.mode_cli = self.node.create_client(SetMode, 'set_mode')
        self.photo_cli = self.node.create_client(TakePhoto, 'take_photo')

        ## DECLARE ACTIONS
        self.node.goto_rel_action_client = ActionClient(self.node, GotoRelative, 'goto_relative')
        self.node.arm_action_client = ActionClient(self.node, Arm, 'Arm')
        self.node.takeoff_action_client = ActionClient(self.node, Takeoff, 'takeoff')

    def ros_send_goto_relative(self, north, east, down):
        self.node.get_logger().info("Sending goto relative action goal")
        goal_msg = GotoRelative.Goal()
        goal_msg.north = north
        goal_msg.east = east
        goal_msg.down = down
        while not self.node.goto_rel_action_client.wait_for_server():
            self.get_logger().info('waiting for goto server...')

        self.node.goto_rel_action_client.send_goal_async(goal_msg)
        self.node.get_logger().info("Goto action sent")


    def gps_timer_update(self):
        self.ros_update_position()

    def timer_ros_update(self):
        rclpy.spin_once(self.node, timeout_sec=0.05)

    def ros_update_position(self):
        request = GetLocationRelative.Request()
        gps_future = self.gps_cli.call_async(request)
        rclpy.spin_until_future_complete(self.node, gps_future, timeout_sec=0.05)
        if gps_future.result() is not None:
            north = round(gps_future.result().north, 2)
            east = round(gps_future.result().east, 2)
            down = round(gps_future.result().down, 2)
            self.textBrowser_2.clear()
            text_msg = " North: " + str(north)
            self.textBrowser_2.append(text_msg)
            text_msg = " East: " + str(east)
            self.textBrowser_2.append(text_msg)
            text_msg = " Down: " + str(down)
            self.textBrowser_2.append(text_msg)

    def log_callback(self, log):
        # self.box_logger.add_log(" " + log.name + ": " + log.msg)
        self.log_textEdit.append(" " + log.name + ": " + log.msg)
        QtWidgets.qApp.processEvents()

    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1289, 858)
        MainWindow.setStyleSheet("background-color: rgb(45,40,80);\n"
                                 "color: rgb(240,240,240);")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.label_4 = QtWidgets.QLabel(self.centralwidget)
        self.label_4.setGeometry(QtCore.QRect(30, 10, 731, 71))
        font = QtGui.QFont()
        font.setPointSize(30)
        font.setBold(True)
        font.setWeight(75)
        self.label_4.setFont(font)
        self.label_4.setAlignment(QtCore.Qt.AlignCenter)
        self.label_4.setObjectName("label_4")
        self.label_5 = QtWidgets.QLabel(self.centralwidget)
        self.label_5.setGeometry(QtCore.QRect(80, 130, 301, 61))
        font = QtGui.QFont()
        font.setPointSize(24)
        font.setBold(True)
        font.setWeight(75)
        self.label_5.setFont(font)
        self.label_5.setAlignment(QtCore.Qt.AlignCenter)
        self.label_5.setObjectName("label_5")
        self.gridLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_2.setGeometry(QtCore.QRect(440, 210, 321, 281))
        self.gridLayoutWidget_2.setObjectName("gridLayoutWidget_2")
        self.gridLayout_2 = QtWidgets.QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.left_button = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        self.left_button.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.left_button.sizePolicy().hasHeightForWidth())
        self.left_button.setSizePolicy(sizePolicy)
        self.left_button.setText("")
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap("/home/stas/Dron/drone-knr/src/drone_gui/icons/left.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.left_button.setIcon(icon)
        self.left_button.setIconSize(QtCore.QSize(20, 20))
        self.left_button.setObjectName("left_button")
        self.gridLayout_2.addWidget(self.left_button, 1, 0, 1, 1)
        self.right_button = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.right_button.sizePolicy().hasHeightForWidth())
        self.right_button.setSizePolicy(sizePolicy)
        self.right_button.setText("")
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("/home/stas/Dron/drone-knr/src/drone_gui/icons/right-arrow-svgrepo-com.svg"), QtGui.QIcon.Normal,
                        QtGui.QIcon.Off)
        self.right_button.setIcon(icon1)
        self.right_button.setObjectName("right_button")
        self.gridLayout_2.addWidget(self.right_button, 1, 2, 1, 1)
        self.back_button = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.back_button.sizePolicy().hasHeightForWidth())
        self.back_button.setSizePolicy(sizePolicy)
        self.back_button.setText("")
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("/home/stas/Dron/drone-knr/src/drone_gui/icons/down.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.back_button.setIcon(icon2)
        self.back_button.setObjectName("back_button")
        self.gridLayout_2.addWidget(self.back_button, 3, 1, 1, 1)
        self.forward_button = QtWidgets.QPushButton(self.gridLayoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.forward_button.sizePolicy().hasHeightForWidth())
        self.forward_button.setSizePolicy(sizePolicy)
        self.forward_button.setText("")
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("/home/stas/Dron/drone-knr/src/drone_gui/icons/up.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.forward_button.setIcon(icon3)
        self.forward_button.setObjectName("forward_button")
        self.gridLayout_2.addWidget(self.forward_button, 0, 1, 1, 1)
        self.log_textEdit = QtWidgets.QTextEdit(self.centralwidget)
        self.log_textEdit.setGeometry(QtCore.QRect(780, 20, 471, 551))
        self.log_textEdit.setStyleSheet("background-color: rgb(40,30,55);\n"
                                        "")
        self.log_textEdit.setObjectName("log_textEdit")
        self.gridLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_3.setGeometry(QtCore.QRect(450, 510, 301, 81))
        self.gridLayoutWidget_3.setObjectName("gridLayoutWidget_3")
        self.gridLayout_3 = QtWidgets.QGridLayout(self.gridLayoutWidget_3)
        self.gridLayout_3.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_3.setObjectName("gridLayout_3")
        self.up_button = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.up_button.sizePolicy().hasHeightForWidth())
        self.up_button.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.up_button.setFont(font)
        self.up_button.setIcon(icon3)
        self.up_button.setObjectName("up_button")
        self.gridLayout_3.addWidget(self.up_button, 0, 0, 1, 1)
        self.down_button = QtWidgets.QPushButton(self.gridLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.down_button.sizePolicy().hasHeightForWidth())
        self.down_button.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(13)
        font.setBold(True)
        font.setWeight(75)
        self.down_button.setFont(font)
        self.down_button.setIcon(icon2)
        self.down_button.setObjectName("down_button")
        self.gridLayout_3.addWidget(self.down_button, 0, 1, 1, 1)
        self.gridLayoutWidget_4 = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget_4.setGeometry(QtCore.QRect(30, 210, 395, 391))
        self.gridLayoutWidget_4.setObjectName("gridLayoutWidget_4")
        self.gridLayout_4 = QtWidgets.QGridLayout(self.gridLayoutWidget_4)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.north_lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.north_lineEdit.sizePolicy().hasHeightForWidth())
        self.north_lineEdit.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.north_lineEdit.setFont(font)
        self.north_lineEdit.setStyleSheet("background-color: rgb(40,30,55);")
        self.north_lineEdit.setInputMask("")
        self.north_lineEdit.setText("")
        self.north_lineEdit.setObjectName("north_lineEdit")
        self.gridLayout_4.addWidget(self.north_lineEdit, 0, 1, 1, 1)
        self.pushButton = QtWidgets.QPushButton(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton.sizePolicy().hasHeightForWidth())
        self.pushButton.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(25)
        font.setBold(True)
        font.setWeight(75)
        self.pushButton.setFont(font)
        self.pushButton.setObjectName("pushButton")
        self.gridLayout_4.addWidget(self.pushButton, 3, 0, 1, 3)
        self.label_3 = QtWidgets.QLabel(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_3.setFont(font)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setIndent(5)
        self.label_3.setObjectName("label_3")
        self.gridLayout_4.addWidget(self.label_3, 2, 0, 1, 1)
        self.down_lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.down_lineEdit.sizePolicy().hasHeightForWidth())
        self.down_lineEdit.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.down_lineEdit.setFont(font)
        self.down_lineEdit.setStyleSheet("background-color: rgb(40,30,55);")
        self.down_lineEdit.setObjectName("down_lineEdit")
        self.gridLayout_4.addWidget(self.down_lineEdit, 2, 1, 1, 1)
        self.label_6 = QtWidgets.QLabel(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label_6.setFont(font)
        self.label_6.setAlignment(QtCore.Qt.AlignCenter)
        self.label_6.setIndent(5)
        self.label_6.setObjectName("label_6")
        self.gridLayout_4.addWidget(self.label_6, 0, 0, 1, 1)
        self.label = QtWidgets.QLabel(self.gridLayoutWidget_4)
        font = QtGui.QFont()
        font.setPointSize(20)
        font.setBold(True)
        font.setWeight(75)
        self.label.setFont(font)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setIndent(5)
        self.label.setObjectName("label")
        self.gridLayout_4.addWidget(self.label, 1, 0, 1, 1)
        self.east_lineEdit = QtWidgets.QLineEdit(self.gridLayoutWidget_4)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.east_lineEdit.sizePolicy().hasHeightForWidth())
        self.east_lineEdit.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(19)
        self.east_lineEdit.setFont(font)
        self.east_lineEdit.setStyleSheet("background-color: rgb(40,30,55);")
        self.east_lineEdit.setText("")
        self.east_lineEdit.setObjectName("lineEdit")
        self.gridLayout_4.addWidget(self.east_lineEdit, 1, 1, 1, 1)
        self.horizontalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setGeometry(QtCore.QRect(450, 610, 301, 51))
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.label_2 = QtWidgets.QLabel(self.horizontalLayoutWidget)
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_2.setFont(font)
        self.label_2.setAlignment(QtCore.Qt.AlignCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout.addWidget(self.label_2)
        self.step_spinBox = QtWidgets.QSpinBox(self.horizontalLayoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.step_spinBox.sizePolicy().hasHeightForWidth())
        self.step_spinBox.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(15)
        self.step_spinBox.setFont(font)
        self.step_spinBox.setStyleSheet("background-color: rgb(40,30,55);")
        self.step_spinBox.setAlignment(QtCore.Qt.AlignCenter)
        self.step_spinBox.setObjectName("step_spinBox")
        self.horizontalLayout.addWidget(self.step_spinBox)
        self.label_7 = QtWidgets.QLabel(self.centralwidget)
        self.label_7.setGeometry(QtCore.QRect(440, 130, 301, 61))
        font = QtGui.QFont()
        font.setPointSize(24)
        font.setBold(True)
        font.setWeight(75)
        self.label_7.setFont(font)
        self.label_7.setAlignment(QtCore.Qt.AlignCenter)
        self.label_7.setObjectName("label_7")
        self.horizontalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_2.setGeometry(QtCore.QRect(30, 610, 391, 51))
        self.horizontalLayoutWidget_2.setObjectName("horizontalLayoutWidget_2")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_2)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.arm_button = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.arm_button.setObjectName("arm_button")
        self.horizontalLayout_2.addWidget(self.arm_button)
        self.takeoff_button = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.takeoff_button.setObjectName("takeoff_button")
        self.horizontalLayout_2.addWidget(self.takeoff_button)
        self.land_button = QtWidgets.QPushButton(self.horizontalLayoutWidget_2)
        self.land_button.setObjectName("land_button")
        self.horizontalLayout_2.addWidget(self.land_button)
        self.horizontalLayoutWidget_3 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_3.setGeometry(QtCore.QRect(780, 580, 471, 87))
        self.horizontalLayoutWidget_3.setObjectName("horizontalLayoutWidget_3")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_3)
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_8 = QtWidgets.QLabel(self.horizontalLayoutWidget_3)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy)
        self.label_8.setMinimumSize(QtCore.QSize(150, 0))
        self.label_8.setBaseSize(QtCore.QSize(0, 0))
        font = QtGui.QFont()
        font.setPointSize(15)
        font.setBold(True)
        font.setWeight(75)
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignCenter)
        self.label_8.setIndent(10)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_4.addWidget(self.label_8)
        self.textBrowser_2 = QtWidgets.QTextBrowser(self.horizontalLayoutWidget_3)
        self.textBrowser_2.setStyleSheet("background-color: rgb(40,30,55);")
        self.textBrowser_2.setObjectName("textBrowser_2")
        self.horizontalLayout_4.addWidget(self.textBrowser_2)
        self.gridLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.gridLayoutWidget.setGeometry(QtCore.QRect(220, 700, 194, 124))
        self.gridLayoutWidget.setObjectName("gridLayoutWidget")
        self.gridLayout = QtWidgets.QGridLayout(self.gridLayoutWidget)
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout.setObjectName("gridLayout")
        self.pwm_spinBox = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.pwm_spinBox.setObjectName("pwm_spinBox")
        self.gridLayout.addWidget(self.pwm_spinBox, 2, 1, 1, 1)
        self.servo_id_spinBox = QtWidgets.QSpinBox(self.gridLayoutWidget)
        self.servo_id_spinBox.setObjectName("servo_id_spinBox")
        self.gridLayout.addWidget(self.servo_id_spinBox, 2, 0, 1, 1)
        self.set_servo = QtWidgets.QPushButton(self.gridLayoutWidget)
        self.set_servo.setObjectName("set_servo")
        self.gridLayout.addWidget(self.set_servo, 3, 0, 1, 2)
        self.label_11 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_11.setObjectName("label_11")
        self.gridLayout.addWidget(self.label_11, 1, 1, 1, 1)
        self.label_10 = QtWidgets.QLabel(self.gridLayoutWidget)
        self.label_10.setObjectName("label_10")
        self.gridLayout.addWidget(self.label_10, 1, 0, 1, 1)
        self.label_9 = QtWidgets.QLabel(self.gridLayoutWidget)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.label_9.setFont(font)
        self.label_9.setAlignment(QtCore.Qt.AlignCenter)
        self.label_9.setObjectName("label_9")
        self.gridLayout.addWidget(self.label_9, 0, 0, 1, 2)
        self.spinBox_4 = QtWidgets.QSpinBox(self.centralwidget)
        self.spinBox_4.setGeometry(QtCore.QRect(660, 830, 76, 29))
        self.spinBox_4.setObjectName("spinBox_4")
        self.horizontalLayoutWidget_4 = QtWidgets.QWidget(self.centralwidget)
        self.horizontalLayoutWidget_4.setGeometry(QtCore.QRect(30, 720, 165, 80))
        self.horizontalLayoutWidget_4.setObjectName("horizontalLayoutWidget_4")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout(self.horizontalLayoutWidget_4)
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.take_photo = QtWidgets.QPushButton(self.horizontalLayoutWidget_4)
        self.take_photo.setObjectName("take_photo")
        self.horizontalLayout_3.addWidget(self.take_photo)
        self.photo_spinBox = QtWidgets.QSpinBox(self.horizontalLayoutWidget_4)
        self.photo_spinBox.setObjectName("photo_spinBox")
        self.horizontalLayout_3.addWidget(self.photo_spinBox)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.init_my_components()
        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

        self.connect_my_signals()
        self.ros_init()
        self.init_box_logger()

    def init_my_components(self):
        self.step_spinBox.setMinimum(0)
        self.step_spinBox.setMaximum(5000)

        self.log_textEdit.setReadOnly(True)

    def connect_my_signals(self):
        self.pushButton.clicked.connect(self.go_button_clicked)
        self.arm_button.clicked.connect(self.arm_button_clicked)
        self.takeoff_button.clicked.connect(self.takeoff_button_clicked)
        self.left_button.clicked.connect(self.left_button_clicked)
        self.up_button.clicked.connect(self.up_button_clicked)
        self.down_button.clicked.connect(self.down_button_clicked)
        self.forward_button.clicked.connect(self.forward_button_clicked)
        self.back_button.clicked.connect(self.back_button_clicked)
        self.right_button.clicked.connect(self.right_button_clicked)
        self.land_button.clicked.connect(self.land_button_clicked)
        self.take_photo.clicked.connect(self.take_photo_button_clicked)
        self.set_servo.clicked.connect(self.set_servo_button_clicked)

        self.step_spinBox.valueChanged.connect(self.step_spinBox_changed)

    def take_photo_button_clicked(self):
        self.take_photo.setStyleSheet("background-color : yellow")
        QtWidgets.qApp.processEvents()

        self.node.get_logger().info("Sending TAKE-PHOTO request")
        photo_num = self.photo_spinBox.value()
        self.ros_send_take_photo(photo_num)

    def set_servo_button_clicked(self):
        self.set_servo.setStyleSheet("background-color : yellow")
        QtWidgets.qApp.processEvents()
        id = self.servo_id_spinBox.value()
        pwm = self.pwm_spinBox.value()
        self.ros_send_servo(id, pwm)

    def step_spinBox_changed(self):
        self.step = float(self.step_spinBox.value() / 100)
        print(self.step)

    def left_button_clicked(self):
        self.ros_send_goto_relative(0.0, -self.step, 0.0)

    def right_button_clicked(self):
        self.ros_send_goto_relative(0.0, self.step, 0.0)

    def forward_button_clicked(self):
        self.ros_send_goto_relative(self.step, 0.0, 0.0)

    def back_button_clicked(self):
        self.ros_send_goto_relative(-self.step, 0.0, 0.0)

    def up_button_clicked(self):
        self.ros_send_goto_relative(0.0, 0.0, -self.step)

    def down_button_clicked(self):
        self.ros_send_goto_relative(0.0, 0.0, self.step)

    def land_button_clicked(self):
        pass

    def takeoff_button_clicked(self):
        self.takeoff_button.setStyleSheet("background-color : yellow")
        QtWidgets.qApp.processEvents()

        altitude = self.down_lineEdit.text() or "2"
        if not altitude.lstrip('-').isdigit():
            self.error_popup()
        else:
            self.ros_send_takeoff(float(altitude))
        self.down_lineEdit.clear()

    def ros_send_servo(self, id, pwm):
        self.node.get_logger().info("Sending SERVO request")
        request = SetServo.Request()
        request.servo_id = id
        request.pwm = pwm
        servo_future = self.servo_cli.call_async(request)
        timeout_ret = rclpy.spin_until_future_complete(self.node, servo_future, timeout_sec=1)
        if timeout_ret:
            self.node.get_logger().info("Servo request finished")
            self.set_servo.setStyleSheet("background-color : green")
        else:
            self.node.get_logger().info("Servo request failed")
            self.set_servo.setStyleSheet("background-color : red")

    def ros_send_take_photo(self, photos_number=0):
        request = TakePhoto.Request()
        request.photos_number = photos_number
        photo_future = self.photo_cli.call_async(request)
        timeout_ret = rclpy.spin_until_future_complete(self.node, photo_future, timeout_sec=3)
        if timeout_ret:
            self.node.get_logger().info("Photo request sucseed")
            self.take_photo.setStyleSheet("background-color : green")
        else:
            self.node.get_logger().info("Photo request failed")
            self.take_photo.setStyleSheet("background-color : red")

    def ros_send_takeoff(self, altitude=2.0):
        self.node.get_logger().info("Sending TAKE-OFF action goal")
        rclpy.spin_once(self.node, timeout_sec=0.05)
        goal_msg = Takeoff.Goal()
        goal_msg.altitude = altitude
        while not self.node.takeoff_action_client.wait_for_server():
            self.get_logger().info('waiting for TAKE-OFF server...')

        self.takeoff_future = self.node.takeoff_action_client.send_goal_async(goal_msg)
        self.takeoff_future.add_done_callback(self.takeoff_response_callback)

    def takeoff_response_callback(self, future):
        goal_handle = future.result()
        self.node.get_logger().info('waiting for takeoff response...')
        self.takeoff_get_result_future = goal_handle.get_result_async()
        self.takeoff_get_result_future.add_done_callback(self.takeoff_get_result_callback)

    def takeoff_get_result_callback(self, future):
        result = future.result().result.result
        print(result)
        if result == 1:
            self.takeoff_button.setStyleSheet("background-color : green")
            self.node.get_logger().info('Takeoff request succesfull')
        else:
            self.takeoff_button.setStyleSheet("background-color : red")
            self.node.get_logger().info('Takeoff request error')

    def arm_button_clicked(self):
        self.arm_button.setStyleSheet("background-color : yellow")
        # Set mode to guided
        self.node.get_logger().info("Sending GUIDED mode request")
        request = SetMode.Request()
        request.mode = "GUIDED"
        mode_future = self.mode_cli.call_async(request)
        rclpy.spin_until_future_complete(self.node, mode_future, timeout_sec=10)
        self.node.get_logger().info("Mode request sucesfull")

        # ARM drone
        self.node.get_logger().info("Sending ARM action goal")

        goal_msg = Arm.Goal()

        while not self.node.arm_action_client.wait_for_server():
            self.get_logger().info('waiting for ARM server...')

        self.arm_future = self.node.arm_action_client.send_goal_async(goal_msg)
        self.arm_future.add_done_callback(self.arm_response_callback)

    def arm_response_callback(self, future):
        goal_handle = future.result()
        self.node.get_logger().info('waiting for arm response...')
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.arm_get_result_callback)

    def arm_get_result_callback(self, future):
        result = future.result().result.result
        print(result)
        if result == 1:
            self.arm_button.setStyleSheet("background-color : green")
            self.node.get_logger().info('Arm request succesfull')
        else:
            self.arm_button.setStyleSheet("background-color : red")
            self.node.get_logger().info('Arm request error')

    def go_button_clicked(self):
        north = self.north_lineEdit.text() or "0"
        east = self.east_lineEdit.text() or "0"
        down = self.down_lineEdit.text() or "0"
        print(north.lstrip('-').replace(".", ""))
        if not north.lstrip('-').replace(".", "").isdigit() or not east.lstrip('-').replace(".", "").isdigit() or not down.lstrip('-').replace(".", "").isdigit():
            self.error_popup()
        else:
            self.ros_send_goto_relative(float(north), float(east), float(down))
        self.east_lineEdit.clear()
        self.down_lineEdit.clear()
        self.north_lineEdit.clear()

    def error_popup(self):
        msg = QMessageBox()
        msg.setWindowTitle("Invalid coordinates value")
        msg.setText("Enter valid coordinates values")
        msg.setIcon(QMessageBox.Warning)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.setDefaultButton(QMessageBox.Ok)

        x = msg.exec_()

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "KNR Drone"))
        self.label_4.setText(_translate("MainWindow", "KNR Drone control"))
        self.label_5.setText(_translate("MainWindow", "Go to position"))
        self.up_button.setText(_translate("MainWindow", " UP"))
        self.down_button.setText(_translate("MainWindow", " DOWN"))
        self.pushButton.setText(_translate("MainWindow", "GO"))
        self.label_3.setText(_translate("MainWindow", "Down (z)"))
        self.label_6.setText(_translate("MainWindow", "North (x)"))
        self.label.setText(_translate("MainWindow", "East (y)"))
        self.label_2.setText(_translate("MainWindow", "Step"))
        self.label_7.setText(_translate("MainWindow", "Move"))
        self.arm_button.setText(_translate("MainWindow", "Arm"))
        self.takeoff_button.setText(_translate("MainWindow", "Take off"))
        self.land_button.setText(_translate("MainWindow", "Land"))
        self.label_8.setText(_translate("MainWindow", "position"))
        self.set_servo.setText(_translate("MainWindow", "Set"))
        self.label_11.setText(_translate("MainWindow", "PWM"))
        self.label_10.setText(_translate("MainWindow", "Id"))
        self.label_9.setText(_translate("MainWindow", "Set servo"))
        self.take_photo.setText(_translate("MainWindow", "Take photos"))


def main():
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QtWidgets.QMainWindow()
    ui = Ui_MainWindow()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
