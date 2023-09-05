#
# Copyright (C) 2016  UAVCAN Development Team  <uavcan.org>
#
# This software is distributed under the terms of the MIT License.
#
# Author: Pavel Kirienko <pavel.kirienko@zubax.com>
#

import os
import re
from functools import partial
import dronecan
from dronecan.driver import CANFrame
from PyQt5.QtWidgets import QMainWindow, QLabel, QWidget, QHBoxLayout, QGroupBox, \
    QVBoxLayout, QPushButton, QFileDialog, QLineEdit, QTextEdit, QCheckBox, QMessageBox, QComboBox
from PyQt5.QtGui import QColor, QIcon, QTextOption, QRegExpValidator
from PyQt5.QtCore import Qt, QTimer, QRegExp
from logging import getLogger
from .. import get_app_icon

MANUAL_CAN_DEV = "cansend can0"

CONTROL_MODE_JOG = "Jog"
CONTROL_MODE_CONTINUAL_SPEEED = "Continual speed"
CONTROL_MODE_RELATIVE_POSITIONING = "Relative positioning"
CONTROL_MODE_ABSOLUTE_POSITIONING = "Absolute positioning"
CONTROL_MODE_SPEED_CYCLING = "Speed cycling"
CONTROL_MODE_POSITION_CYCLING = "Position cycling"

CONTROL_START_PLUS = "Start+"
CONTROL_START_MINUS = "Start-"
CONTROL_STOP = "Stop"

PROTOCOL_TYPE_MAUAL_CAN = "Manual CAN"
PROTOCOL_TYPE_OVER_CAN_BRIDGE = "Over CAN bridge"

logger = getLogger(__name__)

def get_validator_parameters():
    regex = QRegExp(r'^([0-9]{1,10}|429496729[0-5])$')
    return QRegExpValidator(regex)

def get_validator_raw_parameters():
    regex = QRegExp(r'^\d{1,10}$')
    return QRegExpValidator(regex)

class CANManualProtocol():
    CAN_ID_PDO_SPD = "100"
    CAN_ID_PDO_SPD_REP = "101"
    CAN_ID_PDO_POS = "110"
    CAN_ID_PDO_POS_REP = "111"
    CAN_ID_SDO_REC = "601"
    CAN_ID_SDO_TRA = "581"

    CAN_ID_ZEROPOS = "102"
    CAN_IS_CLR_ERR = "103"
    CAN_ID_CLR_ERR_REP = "104"

    def convert_value(self, number):
        return f"{(number >> 24) & 0xFF:02X}.{(number >> 16) & 0xFF:02X}.{(number >> 8) & 0xFF:02X}.{number & 0xFF:02X}"

    def getCommStartContinualSpeed(self, inst, speed_axis1, speed_axis2):
        comm_str = inst + self.CAN_ID_PDO_SPD + "#" + self.convert_value(speed_axis1) + "." + self.convert_value(speed_axis2)
        return comm_str

    def getCommStop(self, inst):
        comm_str = inst + self.CAN_ID_PDO_SPD + "#" + self.convert_value(0) + "." + self.convert_value(0)
        return  comm_str

class TGDrivesControlWindow(QMainWindow):
    DEFAULT_PLOT_X_RANGE = 120
    BUS_LOAD_PLOT_MAX_SAMPLES = 50000

    def __init__(self, get_frame, iface_name):
        super(TGDrivesControlWindow, self).__init__()
        self.setWindowTitle('TGDrives control (%s)' % iface_name.split(os.path.sep)[-1])
        self.setWindowIcon(get_app_icon())

        self.setGeometry(100, 100, 700, 300)

        self._getFrame = get_frame

        self.motion_type = CONTROL_MODE_JOG
        self.control_acc_dec = 0.0
        # self.control_speed = 0.0
        self.control_speed = 0
        self.control_speed2 = 0.0
        self.control_position = 0.0
        self.control_position2 = 0.0
        self.control_time = 0.0
        self.control_time2 = 0.0
        self.control_delay = 0.0

        header_group_box = QGroupBox("Main Options")
        header_group_layout = QHBoxLayout(header_group_box)
        main_info_layout = QVBoxLayout(self)
        main_buttons_layout = QVBoxLayout(self)

        self.servo_type_lable_name = QLabel("Servo type:")
        self.servo_type_lable_container = QLabel("TGZS-48-100-250-CAN")
        self.servo_type_lable_container.setStyleSheet("color: blue; margin-left: 10px")
        self.servo_hw_version_name = QLabel("HW version:")
        self.servo_hw_version_container = QLabel("0.0.4.1")
        self.servo_hw_version_container.setStyleSheet("color: blue; margin-left: 10px")
        self.servo_serial_number_lable_name = QLabel("Serial number:")
        self.servo_serial_number_lable_container = QLabel("000.000.000.000")
        self.servo_serial_number_lable_container.setStyleSheet("color: blue; margin-left: 10px")
        
        self.servo_enable_button = QPushButton("Enable", self)
        self.servo_enable_button.clicked.connect(self.handler_enable_servo)
        self.servo_disable_button = QPushButton("Disable", self)
        self.servo_disable_button.clicked.connect(self.handler_disable_servo)

        main_info_layout.addWidget(self.servo_type_lable_name)
        main_info_layout.addWidget(self.servo_type_lable_container)
        main_info_layout.addWidget(self.servo_hw_version_name)
        main_info_layout.addWidget(self.servo_hw_version_container)
        main_info_layout.addWidget(self.servo_serial_number_lable_name)
        main_info_layout.addWidget(self.servo_serial_number_lable_container)

        main_buttons_layout.addWidget(self.servo_enable_button)
        main_buttons_layout.addWidget(self.servo_disable_button)

        header_group_layout.addLayout(main_info_layout)
        header_group_layout.addLayout(main_buttons_layout)

        control_layout = QHBoxLayout(self)
        control_group_parameters = QGroupBox("Parameters")
        control_parameters_layout = QVBoxLayout(control_group_parameters)
        control_group_adv_parameters = QGroupBox("Adv. Parameters")
        # control_group_adv_parameters.setVisible(False)
        control_adv_parameters_layout = QVBoxLayout(control_group_adv_parameters)
        control_group_buttons = QGroupBox("Control")
        control_buttons_layout = QVBoxLayout(control_group_buttons)
        
        control_item_protocol_type = QHBoxLayout(self)
        self.control_protocol_type_lable = QLabel("Protocol type:")
        self.control_protocol_type_dropdown = QComboBox(self)
        self.control_protocol_type_dropdown.setMaximumWidth(280)
        self.control_protocol_type_dropdown.addItem(PROTOCOL_TYPE_MAUAL_CAN)
        self.control_protocol_type_dropdown.addItem(PROTOCOL_TYPE_OVER_CAN_BRIDGE)
        self.control_protocol_type_dropdown.currentIndexChanged.connect(self.protocol_changed)
        control_item_protocol_type.addWidget(self.control_protocol_type_lable)
        control_item_protocol_type.addWidget(self.control_protocol_type_dropdown)
        control_parameters_layout.addLayout(control_item_protocol_type)

        control_item_motion_type = QHBoxLayout(self)
        self.control_motion_type_lable = QLabel("Motion type:")
        self.control_motion_type_dropdown = QComboBox(self)
        self.control_motion_type_dropdown.setMaximumWidth(280)
        self.control_motion_type_dropdown.addItem(CONTROL_MODE_JOG)
        self.control_motion_type_dropdown.addItem(CONTROL_MODE_CONTINUAL_SPEEED)
        self.control_motion_type_dropdown.addItem(CONTROL_MODE_RELATIVE_POSITIONING)
        self.control_motion_type_dropdown.addItem(CONTROL_MODE_ABSOLUTE_POSITIONING)
        self.control_motion_type_dropdown.addItem(CONTROL_MODE_SPEED_CYCLING)
        self.control_motion_type_dropdown.addItem(CONTROL_MODE_POSITION_CYCLING)
        self.control_motion_type_dropdown.currentIndexChanged.connect(self.mode_changed)
        # self.control_motion_type_dropdown.addItem("Electronic gear")
        control_item_motion_type.addWidget(self.control_motion_type_lable)
        control_item_motion_type.addWidget(self.control_motion_type_dropdown)
        control_parameters_layout.addLayout(control_item_motion_type)

        control_item_acc_dec = QHBoxLayout(self)
        self.control_acc_dec_lable = QLabel("Acc, Dec:")
        self.control_acc_dec_edit = QLineEdit(self)
        self.control_acc_dec_edit.setValidator(get_validator_parameters())
        self.control_acc_dec_edit.setText("100.000")
        self.control_acc_dec_edit.setFixedWidth(100)
        self.control_acc_dec_edit.setMaxLength(7)
        self.control_acc_dec_info = QLabel("rev/s^2")
        self.control_acc_dec_info.setMaximumWidth(70)
        control_item_acc_dec.addWidget(self.control_acc_dec_lable)
        control_item_acc_dec.addWidget(self.control_acc_dec_edit)
        control_item_acc_dec.addWidget(self.control_acc_dec_info)
        control_parameters_layout.addLayout(control_item_acc_dec)

        control_item_speed = QHBoxLayout(self)
        self.control_speed_lable = QLabel("Speed:")
        self.control_speed_edit = QLineEdit(self)
        # self.control_speed_edit.setValidator(get_validator_parameters())
        # self.control_speed_edit.setText("0.500")
        self.control_speed_edit.setValidator(get_validator_raw_parameters())
        self.control_speed_edit.setText("17825791")
        self.control_speed_edit.setFixedWidth(100)
        self.control_acc_dec_edit.setMaxLength(7)
        # self.control_speed_info = QLabel("rev/s")
        self.control_speed_info = QLabel("?")
        self.control_speed_info.setMaximumWidth(70)
        control_item_speed.addWidget(self.control_speed_lable)
        control_item_speed.addWidget(self.control_speed_edit)
        control_item_speed.addWidget(self.control_speed_info)
        control_parameters_layout.addLayout(control_item_speed)

        control_item_position = QHBoxLayout(self)
        self.control_position_lable = QLabel("Possition:")
        self.control_position_edit = QLineEdit(self)
        self.control_position_edit.setValidator(get_validator_parameters())
        self.control_position_edit.setFixedWidth(100)
        self.control_acc_dec_edit.setMaxLength(7)
        self.control_position_info = QLabel("rev")
        self.control_position_info.setMaximumWidth(70)
        control_item_position.addWidget(self.control_position_lable)
        control_item_position.addWidget(self.control_position_edit)
        control_item_position.addWidget(self.control_position_info)
        control_parameters_layout.addLayout(control_item_position)

        control_item_time = QHBoxLayout(self)
        self.control_time_lable = QLabel("Time:")
        self.control_time_edit = QLineEdit(self)
        self.control_time_edit.setValidator(get_validator_parameters())
        self.control_time_edit.setFixedWidth(100)
        self.control_acc_dec_edit.setMaxLength(7)
        self.control_time_info = QLabel("s")
        self.control_time_info.setMaximumWidth(70)
        control_item_time.addWidget(self.control_time_lable)
        control_item_time.addWidget(self.control_time_edit)
        control_item_time.addWidget(self.control_time_info)
        control_parameters_layout.addLayout(control_item_time)

        control_item_delay = QHBoxLayout(self)
        self.control_delay_lable = QLabel("Delay:")
        self.control_delay_edit = QLineEdit(self)
        self.control_delay_edit.setValidator(get_validator_parameters())
        self.control_delay_edit.setFixedWidth(100)
        self.control_acc_dec_edit.setMaxLength(7)
        self.control_delay_info = QLabel("s")
        self.control_delay_info.setMaximumWidth(70)
        control_item_delay.addWidget(self.control_delay_lable)
        control_item_delay.addWidget(self.control_delay_edit)
        control_item_delay.addWidget(self.control_delay_info)
        control_adv_parameters_layout.addLayout(control_item_delay)

        control_item_speed2 = QHBoxLayout(self)
        self.control_speed2_lable = QLabel("Speed 2:")
        self.control_speed2_edit = QLineEdit(self)
        self.control_speed2_edit.setValidator(get_validator_parameters())
        self.control_speed2_edit.setFixedWidth(100)
        self.control_acc_dec_edit.setMaxLength(7)
        self.control_speed2_info = QLabel("rev/s")
        self.control_speed2_info.setMaximumWidth(70)
        control_item_speed2.addWidget(self.control_speed2_lable)
        control_item_speed2.addWidget(self.control_speed2_edit)
        control_item_speed2.addWidget(self.control_speed2_info)
        control_adv_parameters_layout.addLayout(control_item_speed2)

        control_item_time2 = QHBoxLayout(self)
        self.control_time2_lable = QLabel("Time 2:")
        self.control_time2_edit = QLineEdit(self)
        self.control_time2_edit.setValidator(get_validator_parameters())
        self.control_time2_edit.setFixedWidth(100)
        self.control_acc_dec_edit.setMaxLength(7)
        self.control_time2_info = QLabel("s")
        self.control_time2_info.setMaximumWidth(70)
        control_item_time2.addWidget(self.control_time2_lable)
        control_item_time2.addWidget(self.control_time2_edit)
        control_item_time2.addWidget(self.control_time2_info)
        control_adv_parameters_layout.addLayout(control_item_time2)

        control_item_position2 = QHBoxLayout(self)
        self.control_position2_lable = QLabel("Possition 2:")
        self.control_position2_edit = QLineEdit(self)
        self.control_position2_edit.setValidator(get_validator_parameters())
        self.control_position2_edit.setFixedWidth(100)
        self.control_acc_dec_edit.setMaxLength(7)
        self.control_position2_info = QLabel("rev")
        self.control_position2_info.setMaximumWidth(70)
        control_item_position2.addWidget(self.control_position2_lable)
        control_item_position2.addWidget(self.control_position2_edit)
        control_item_position2.addWidget(self.control_position2_info)
        control_adv_parameters_layout.addLayout(control_item_position2)
        control_item_delay.activate()


        self.control_start_plus_button = QPushButton("Start +", self)
        self.control_start_plus_button.clicked.connect(self.handler_start_servo_plus)
        self.control_start_minus_button = QPushButton("Start -", self)
        self.control_start_minus_button.clicked.connect(self.handler_start_servo_minus)
        self.control_stop_button = QPushButton("Stop", self)
        self.control_stop_button.clicked.connect(self.handler_stop)
        control_buttons_layout.addWidget(self.control_start_plus_button)
        control_buttons_layout.addWidget(self.control_start_minus_button)
        control_buttons_layout.addWidget(self.control_stop_button)

        self.hide_control_widgets()


        control_layout.addWidget(control_group_parameters)
        control_layout.addWidget(control_group_adv_parameters)
        control_layout.addWidget(control_group_buttons)



        layout = QVBoxLayout(self)
        layout.addWidget(header_group_box)
        layout.addLayout(control_layout)

        self.log_text_edit = QTextEdit(self)
        self.log_text_edit.setReadOnly(True)
        layout.addWidget(self.log_text_edit)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

    def hide_control_widgets(self):
        self.control_position_lable.setVisible(False)
        self.control_position_edit.setVisible(False)
        self.control_position_info.setVisible(False)
        self.control_position2_lable.setVisible(False)
        self.control_position2_edit.setVisible(False)
        self.control_position2_info.setVisible(False)
        self.control_time_lable.setVisible(False)
        self.control_time_edit.setVisible(False)
        self.control_time_info.setVisible(False)
        self.control_time2_lable.setVisible(False)
        self.control_time2_edit.setVisible(False)
        self.control_time2_info.setVisible(False)
        self.control_delay_lable.setVisible(False)
        self.control_delay_edit.setVisible(False)
        self.control_delay_info.setVisible(False)
        self.control_speed2_lable.setVisible(False)
        self.control_speed2_edit.setVisible(False)
        self.control_speed2_info.setVisible(False)
    
    def handler_jog(self):
        self.log_text_edit.append(f"Mode jog")
        self.motion_type = CONTROL_MODE_JOG
        self.hide_control_widgets()
        return
    
    def handler_continual_speed(self):
        self.log_text_edit.append(f"Mode Continual speed")
        self.motion_type = CONTROL_MODE_CONTINUAL_SPEEED
        self.hide_control_widgets()
        return
    
    def handler_relative_positioning(self):
        self.log_text_edit.append(f"Mode Relative positioning")
        self.motion_type = CONTROL_MODE_RELATIVE_POSITIONING
        self.hide_control_widgets()
        self.control_position_lable.setVisible(True)
        self.control_position_edit.setVisible(True)
        self.control_position_info.setVisible(True)
        return
    
    def handler_absolute_positioning(self):
        self.log_text_edit.append(f"Mode Absolute positioning")
        self.motion_type = CONTROL_MODE_ABSOLUTE_POSITIONING
        self.hide_control_widgets()
        self.control_position_lable.setVisible(True)
        self.control_position_edit.setVisible(True)
        self.control_position_info.setVisible(True)
        return
    
    def handler_speed_cycling(self):
        self.log_text_edit.append(f"Mode Speed cycling")
        self.motion_type = CONTROL_MODE_SPEED_CYCLING
        self.hide_control_widgets()
        self.control_time_lable.setVisible(True)
        self.control_time_edit.setVisible(True)
        self.control_time_info.setVisible(True)
        self.control_time2_lable.setVisible(True)
        self.control_time2_edit.setVisible(True)
        self.control_time2_info.setVisible(True)
        self.control_delay_lable.setVisible(True)
        self.control_delay_edit.setVisible(True)
        self.control_delay_info.setVisible(True)
        self.control_speed2_lable.setVisible(True)
        self.control_speed2_edit.setVisible(True)
        self.control_speed2_info.setVisible(True)
        return
    
    def handler_position_cycling(self):
        self.log_text_edit.append(f"Mode Position cycling")
        self.motion_type = CONTROL_MODE_POSITION_CYCLING
        self.hide_control_widgets()
        self.control_position_lable.setVisible(True)
        self.control_position_edit.setVisible(True)
        self.control_position_info.setVisible(True)
        self.control_position2_lable.setVisible(True)
        self.control_position2_edit.setVisible(True)
        self.control_position2_info.setVisible(True)
        self.control_delay_lable.setVisible(True)
        self.control_delay_edit.setVisible(True)
        self.control_delay_info.setVisible(True)
        return

    def fill_control_config(self):
        try:
            self.control_acc_dec = float(self.control_acc_dec_edit.text())
        except ValueError:
            self.log_text_edit.append("Invalid Custom Input value control_acc_dec.")
        try:
            self.control_speed = int(self.control_speed_edit.text())
        except ValueError:
            self.log_text_edit.append("Invalid Custom Input value control_speed.")
        try:
            self.control_speed2 = float(self.control_speed2_edit.text())
        except ValueError:
            self.log_text_edit.append("Invalid Custom Input value control_speed2.")
        try:
            self.control_position = float(self.control_position_edit.text())
        except ValueError:
            self.log_text_edit.append("Invalid Custom Input value control_position.")
        try:
            self.control_position2 = float(self.control_position2_edit.text())
        except ValueError:
            self.log_text_edit.append("Invalid Custom Input value control_position2.")
        try:
            self.control_time = float(self.control_time_edit.text())
        except ValueError:
            self.log_text_edit.append("Invalid Custom Input value control_time.")
        try:
            self.control_time2 = float(self.control_time2_edit.text())
        except ValueError:
            self.log_text_edit.append("Invalid Custom Input value control_time2.")
        try:
            self.control_delay = float(self.control_delay_edit.text())
        except ValueError:
            self.log_text_edit.append("Invalid Custom Input value control_delay.")
            
        self.log_text_edit.append(f"control_acc_dec value: {self.control_acc_dec}")
        self.log_text_edit.append(f"control_speed value: {self.control_speed}")
        self.log_text_edit.append(f"control_speed2 value: {self.control_speed2}")
        self.log_text_edit.append(f"control_position value: {self.control_position}")
        self.log_text_edit.append(f"control_position2 value: {self.control_position2}")
        self.log_text_edit.append(f"control_time value: {self.control_time}")
        self.log_text_edit.append(f"control_time2 value: {self.control_time2}")
        self.log_text_edit.append(f"control_delay value: {self.control_delay}")

    def send_command(self, control):
        inst = "cansend can0 "
        command = ""
        mode_com = ""
        speed = self.control_speed
        protocol = CANManualProtocol()

        if self.motion_type == CONTROL_MODE_CONTINUAL_SPEEED:
            mode_com = "100"
        else:
            self.log_text_edit.append(f"ERROR: Unsupported mode")
            return
        if control == CONTROL_START_PLUS:
            command = protocol.getCommStartContinualSpeed(inst, speed, 0)
        elif control == CONTROL_START_MINUS:
            command = protocol.getCommStartContinualSpeed(inst, 0 - speed, 0)
        elif control == CONTROL_STOP:
            command = protocol.getCommStop(inst)
            

        if command != "":
            os.system(command)
            self.log_text_edit.append(f"cmd executed Successful: {command}")
        else:
            self.log_text_edit.append(f"Error cmd")

    def handler_enable_servo(self):
        self.log_text_edit.append(f"Button Enable servo pressed")
        item = self._getFrame()
        direction, frame = item
        print("dir: " + direction + " frame: " + frame)

    def handler_disable_servo(self):
        self.log_text_edit.append(f"Button Disable servo pressed")

    def handler_start_servo_plus(self):
        self.log_text_edit.append(f"Button Start+ pressed")
        self.fill_control_config()
        self.send_command(CONTROL_START_PLUS)
        
    def handler_start_servo_minus(self):
        self.log_text_edit.append(f"Button Start- pressed")
        self.fill_control_config()
        self.send_command(CONTROL_START_MINUS)

    def handler_stop(self):
        self.log_text_edit.append(f"Button Stop pressed")
        self.send_command(CONTROL_STOP)

    control_modes_map = {
        CONTROL_MODE_JOG: handler_jog,
        CONTROL_MODE_CONTINUAL_SPEEED: handler_continual_speed,
        CONTROL_MODE_RELATIVE_POSITIONING: handler_relative_positioning,
        CONTROL_MODE_ABSOLUTE_POSITIONING: handler_absolute_positioning,
        CONTROL_MODE_SPEED_CYCLING: handler_speed_cycling,
        CONTROL_MODE_POSITION_CYCLING: handler_position_cycling,
    }

    def mode_changed(self, index):
        mode = self.control_motion_type_dropdown.currentText()
        # self.log_text_edit.append(f"Mode changed to: {mode}")
        if mode in self.control_modes_map:
            self.control_modes_map[mode](self)

    def protocol_changed(self, index):
        protocol = self.control_protocol_type_dropdown.currentText()
        self.log_text_edit.append(f"Protocol changed to: {protocol}")

         

    
    

