

##################################################################################################################
"""
"""

# Built-in/Generic Imports
import sys

# Libs
from PyQt5 import QtWidgets
from PyQt5.QtGui import QIcon

import sys
import rclpy
from rclpy.node import Node
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import JointState, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from utils.msg import Goal
from rclpy.qos import QoSProfile
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import matplotlib
matplotlib.use('Qt5Agg')

# Own modules
from .UI_singletons import Ui_singleton

##################################################################################################################


class RLB_viz(Node):
    def __init__(self):
        Node.__init__(self, 'RLB_viz')

        app = QtWidgets.QApplication([])

        # ==================================================== Load GUI
        self.ui = Ui_singleton().interface
        # self.ui.setWindowIcon(QIcon("src/Data/Assets/logo-swan.png"))
        self.ui.setWindowTitle("RLB viz")
        self.ui.showMaximized()

        # ==================================================== Create publishers
        # ----------------------------------- Instruction publishers
        qos = QoSProfile(depth=10)

        self.instruction_publishers = []

        # ==================================================== Create subscribers
        # ----------------------------------- Goal subscribers
        self.goal_subscriber = []

        # ----------------------------------- Pose subscribers
        self.pose_subscribers = []

        # ----------------------------------- Lazer scan subscribers
        self.lazer_scan_subscribers = []

        # -> Display windows
        self.ui.show()

        print("\n - RLB viz Initialisation: Success \n")

        sys.exit(app.exec())


def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    rlb_viz = RLB_viz()

    rclpy.spin(rlb_viz)

    rlb_viz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()