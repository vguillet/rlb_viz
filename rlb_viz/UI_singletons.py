
##################################################################################################################
"""
"""

# Built-in/Generic Imports
import os

# Libs
from PyQt5 import uic

# Own modules

##################################################################################################################


class Singleton(type):
    __instances = {}

    def __call__(cls, *args, **kwargs):
        if cls not in cls.__instances:
            cls.__instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls.__instances[cls]


class Ui_singleton(metaclass=Singleton):
    def __init__(self):
        root_path = str(os.getcwd())
        self.interface = uic.loadUi(root_path + "/ros2_ws/src/rlb_viz/rlb_viz/UIs/Main_v0.0.1.ui")
