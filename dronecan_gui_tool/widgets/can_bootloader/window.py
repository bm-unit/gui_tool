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
from PyQt5.QtWidgets import QMainWindow, QLabel, QWidget, QHBoxLayout, \
    QVBoxLayout, QPushButton, QFileDialog, QLineEdit, QTextEdit, QCheckBox, QMessageBox
from PyQt5.QtGui import QColor, QIcon, QTextOption
from PyQt5.QtCore import Qt, QTimer
from logging import getLogger
from .. import get_app_icon


logger = getLogger(__name__)


class CANBootloaderWindow(QMainWindow):
    DEFAULT_PLOT_X_RANGE = 120
    BUS_LOAD_PLOT_MAX_SAMPLES = 50000

    def __init__(self, get_frame, iface_name):
        super(CANBootloaderWindow, self).__init__()
        self.setWindowTitle('CAN Bootloader (%s)' % iface_name.split(os.path.sep)[-1])
        self.setWindowIcon(get_app_icon())

        self.setGeometry(100, 100, 700, 300)

        file_choose_layout = QHBoxLayout(self)

        self.file_name_line_edit = QLineEdit(self)
        self.file_name_line_edit.setReadOnly(True)
        self.choose_file_button = QPushButton("Choose File", self)
        self.choose_file_button.clicked.connect(self.choose_file)
        file_choose_layout.addWidget(self.file_name_line_edit)
        file_choose_layout.addWidget(self.choose_file_button)

        addr_choose_layout = QHBoxLayout(self)

        self.start_address_lable = QLabel(": Start address (e.g. 0x0800000)")
        self.start_address_edit = QLineEdit(self)
        self.start_address_edit.setMaxLength(10)
        self.start_address_edit.setMaximumWidth(150)
        addr_choose_layout.addWidget(self.start_address_edit)
        addr_choose_layout.addWidget(self.start_address_lable)

        function_button_layout = QHBoxLayout(self)

        self.erase_button = QPushButton("Chip erase", self)
        self.erase_button.clicked.connect(self.chip_erase)
        self.upload_button = QPushButton("Upload", self)
        self.upload_button.clicked.connect(self.upload_file)
        function_button_layout.addWidget(self.erase_button)
        function_button_layout.addWidget(self.upload_button)

        self.use_address_checkbox = QCheckBox("Use default Start address 0x08000000", self)
        self.use_address_checkbox.stateChanged.connect(self.toggle_address_edit)

        layout = QVBoxLayout(self)
        layout.addLayout(file_choose_layout)
        layout.addWidget(self.use_address_checkbox)
        layout.addLayout(addr_choose_layout)
        layout.addLayout(function_button_layout)

        self.log_text_edit = QTextEdit(self)
        self.log_text_edit.setReadOnly(True)
        layout.addWidget(self.log_text_edit)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        # Calling directly from the constructor gets you wrong size information
        # noinspection PyCallByClass,PyTypeChecker
        # QTimer.singleShot(500, self._update_widget_sizes)

    def toggle_address_edit(self):
        self.start_address_edit.setVisible(not self.use_address_checkbox.isChecked())
        self.start_address_lable.setVisible(not self.use_address_checkbox.isChecked())

    def choose_file(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Choose a FW file", "", "Hex/Bin Files (*.hex *.bin);;All Files (*)", options=options)
        
        if file_name:
            self.file_name_line_edit.setText(file_name)
            self.log_text_edit.append(f"Selected file: {file_name}")
        else:
            self.log_text_edit.append("No file selected.")

    def validate_address(self, address):
        # Regular expression to match hexadecimal address format
        pattern = r'^0x[0-9A-Fa-f]+$'
        return re.match(pattern, address)

    def upload_file(self):
        file_name = self.file_name_line_edit.text()
        command = ""
        if os.path.exists(file_name):
            # Check the file extension
            _, file_extension = os.path.splitext(file_name)
            if file_extension in ('.hex'):
                # self.log_text_edit.append(f"File extension is hex: {file_extension}")
                command = "canprog stm32 write " + file_name
            elif file_extension in ('.bin'):
                # self.log_text_edit.append(f"File extension is bin: {file_extension}")
                command = "canprog -f bin stm32 write " + file_name
            else:
                self.log_text_edit.append(f"Invalid file extension: {file_extension}")

            if not self.use_address_checkbox.isChecked():
                address = self.start_address_edit.text()
                if self.validate_address(address):
                    # address_int = int(address, 16)  # Convert hexadecimal string to integer
                    command += " -a " + address
                    # self.log_text_edit.append(f"Address: {address_int} is valid")
                    # Implement your upload logic here
                else:
                    command = ""
                    self.log_text_edit.append(f"Invalid address: {address}")
                    QMessageBox.critical(self, "Error", "Invalid address format. Please use '0x' followed by hexadecimal digits.")
            else:
                self.log_text_edit.append("Start address by default 0x08000000")
                # Implement your upload logic here without address


            if command != "":
                self.log_text_edit.append(f"Uploading file: {file_name} ...")
                os.system(command)
                self.log_text_edit.append(f"cmd executed Successful: {command}")
            else:
                self.log_text_edit.append(f"Error: cmd wrong!")
            
        else:
            self.log_text_edit.append("File not found.")

    def chip_erase(self):
        command = "canprog stm32 erase -P 0 1 2 3"
        self.log_text_edit.append("Chip erase process...")
        os.system(command)
        self.log_text_edit.append(f"cmd executed Successful: {command}")

