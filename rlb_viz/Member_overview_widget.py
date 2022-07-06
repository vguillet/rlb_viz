import os
from PyQt5 import uic
from PyQt5.QtWidgets import QVBoxLayout, QWidget


class Member_overview_widget(QWidget):
    def __init__(self):
        super().__init__()

        root_path = str(os.getcwd())
        self.ui = uic.loadUi(root_path + "/ros2_ws/src/rlb_viz/rlb_viz/UIs/Robot_overview_v0.0.1.ui")
