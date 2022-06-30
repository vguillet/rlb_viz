

##################################################################################################################
"""
"""

# Built-in/Generic Imports
import json

# Libs
import PyQt5
from PyQt5.QtCore import *
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, LaserScan
from rlb_utils.msg import TeamComm
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math

import matplotlib
import matplotlib.patches as mpatches

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar

from matplotlib.figure import Figure
import matplotlib.patheffects as path_effects

from functools import partial

# Own modules
from .UI_singletons import Ui_singleton
from .Member_overview_widget import Member_overview_widget
from .Blit_manager import BlitManager


##################################################################################################################


class RLB_viz_gui():
    def __init__(self):
        # ==================================================== Load GUI
        app = QtWidgets.QApplication([])

        # -> Load gui singleton
        self.ui = Ui_singleton().interface
        
        # -> Connect buttons
        self.ui.add_robot.clicked.connect(self.__manual_add_robot)

        # ==================================================== Create visualiser
        # -> Create canvas widget
        self.sc = MplCanvas(self,
                            width=2,
                            height=2,
                            dpi=100)

        # -> Create blit manager
        self.bm = BlitManager(canvas=self.sc.fig.canvas)

        # -> Add plot view
        self.ui.main_layout.addWidget(self.sc)

        # -> Display windows
        self.ui.show()

        # -> Create plotter timer
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.plot_robots)
        self.plot_timer.start(10)
        
        # -> Create canvas tools widget
        toolbar = NavigationToolbar(self.sc, self.ui)
        self.ui.main_layout.addWidget(toolbar)

        # ==================================================== Create Node
        self.node = Node("RLB_viz")

        # -> QT timer based spin
        self.spin_timer = QtCore.QTimer()
        self.spin_timer.timeout.connect(self.__node_spinner)
        self.spin_timer.start(1)

        # ==================================================== Setup storage variables
        self.team_members = {}
        
        # ==================================================== Create publishers


        # ==================================================== Create subscribers
        # ----------------------------------- Team communications subscriber
        qos = QoSProfile(depth=10)

        self.team_comms_subscriber = self.node.create_subscription(
            msg_type=TeamComm,
            topic="/Team_comms",
            callback=self.team_msg_subscriber_callback,
            qos_profile=qos
            )

        # ----------------------------------- Team communications publisher
        qos = QoSProfile(depth=10)

        self.team_comms_publisher = self.node.create_publisher(
            msg_type=TeamComm,
            topic="/Team_comms",
            qos_profile=qos
            )
        
        # -> To remove
        self.placeholder_callback()

        # ----------------------------------- Goal subscribers
        self.goal_subscriber = []

        # ==================================================== Final setup
        # -> Spin once
        rclpy.spin_once(self.node)

        sys.exit(app.exec())

    # ================================================= Custom ROS2 integration
    def __node_spinner(self):
        # -> Spin once
        rclpy.spin_once(self.node)

    # ================================================= Callbacks definition
    # ---------------------------------- Publishers
    def placeholder_callback(self):
        # -> Artificial add bots
        for i in range(1):
            msg = TeamComm()
            msg.robot_id = f"Turtle_{i+1}"
            msg.type = "Initial"
            msg.memo = ""

            self.add_robot(msg=msg)

    # ---------------------------------- Subscribers
    def team_msg_subscriber_callback(self, msg):
        if msg.robot_id not in self.team_members.keys():
            self.add_robot(msg=msg)

        elif msg.type == "Goal_annoucement":
            # -> Save goal to member's dict
            goal = json.load(msg.memo)

            self.team_members[msg.robot_id]["goal"]["id"] = goal["id"]

            sequence_xs = []
            sequence_ys = []

            for point in goal["sequence"]:
                sequence_xs.append(point.x)
                sequence_ys.append(point.y)

            self.team_members[msg.robot_id]["goal"]["x"] = sequence_xs
            self.team_members[msg.robot_id]["goal"]["y"] = sequence_ys

            self.team_members[msg.robot_id]["goal"]["current_subgoal"] = 0

            # -> Set widget view
            self.team_members[msg.robot_id]["overview_widget"].ui.goal_id.setText(goal["id"])
            self.team_members[msg.robot_id]["overview_widget"].ui.goal_x.setText(str(round(sequence_xs[0], 2)))
            self.team_members[msg.robot_id]["overview_widget"].ui.goal_y.setText(str(round(sequence_ys[0], 2)))
        
        elif msg.type == "Subgoal_completed":
            self.team_members[msg.robot_id]["goal"]["current_subgoal"] += 1

        elif msg.type == "Collision":
           self.team_members[msg.robot_id]["on_collision_course"] = bool(msg.memo) 

    def pose_subscriber_callback(self, robot_id, msg):
        # -> Update position
        self.team_members[robot_id]["pose"]["x"] = msg.pose.position.x
        self.team_members[robot_id]["pose"]["y"] = msg.pose.position.y
        self.team_members[robot_id]["pose"]["z"] = msg.pose.position.z

        self.team_members[robot_id]["overview_widget"].ui.pose_x.setText("{:10.3f}".format(round(msg.pose.position.x, 3)))
        self.team_members[robot_id]["overview_widget"].ui.pose_y.setText("{:10.3f}".format(round(msg.pose.position.y, 3)))
        self.team_members[robot_id]["overview_widget"].ui.pose_z.setText("{:10.3f}".format(round(msg.pose.position.z, 3)))

        # -> Update orientation
        u, v, w = self.__euler_from_quaternion(quat=msg.pose.orientation)

        self.team_members[robot_id]["pose"]["u"] = u
        self.team_members[robot_id]["pose"]["v"] = v
        self.team_members[robot_id]["pose"]["w"] = w

        self.team_members[robot_id]["overview_widget"].ui.pose_u.setText("{:10.3f}".format(round(u, 3)))
        self.team_members[robot_id]["overview_widget"].ui.pose_v.setText("{:10.3f}".format(round(v, 3)))
        self.team_members[robot_id]["overview_widget"].ui.pose_w.setText("{:10.3f}".format(round(w, 3)))

    def lazer_scan_subscriber_callback(self, robot_id, msg):
        scan = list(msg.ranges)
        
        range_min = msg.range_min
        range_max = msg.range_max

        # -> Clean up ranges
        for i, range_measure in enumerate(scan):
            if range_measure < range_min or range_measure > range_max:
                scan[i] = None

        self.team_members[robot_id]["lazer_scan"] = scan

    # ================================================= Utils
    def __manual_add_robot(self):
        msg = TeamComm()
        msg.robot_id = self.ui.add_robot_text_entry.text()
        msg.type = "Initial"
        msg.memo = ""

        # -> Add robot if not already added
        if msg.robot_id not in self.team_members.keys():
            self.add_robot(msg=msg)

    def plot_robots(self):
        for robot_id in self.team_members.keys():
            x = round(self.team_members[robot_id]["pose"]["x"], 3)
            y = round(self.team_members[robot_id]["pose"]["y"], 3)

            # -> Update pose
            self.team_members[robot_id]["pose_artist"].set_xdata(x)
            self.team_members[robot_id]["pose_artist"].set_ydata(y)

            # -> Update goal
            goal_x = self.team_members[robot_id]["goal"]["x"]
            goal_y = self.team_members[robot_id]["goal"]["y"]

            subgoal_index = self.team_members[robot_id]["goal"]["current_subgoal"]

            self.team_members[robot_id]["goal_artist"].set_data(
                goal_x,
                goal_y
                )

            # -> Update goal ray
            if not subgoal_index > len(goal_x) and goal_x:
                self.team_members[robot_id]["goal_ray_artist"].set_xdata([x, goal_x[subgoal_index]])

                self.team_members[robot_id]["goal_ray_artist"].set_ydata([y, goal_y[subgoal_index]])

            else:
                self.team_members[robot_id]["goal_ray_artist"].set_xdata([])
                self.team_members[robot_id]["goal_ray_artist"].set_ydata([])


            # -> Update annoation position
            self.team_members[robot_id]["label_artist"].xy = (x, y)
            self.team_members[robot_id]["label_artist"].set_position((x+0.05, y+0.1))

            # -> Update scan circle position
            self.team_members[robot_id]["scan_circle_artist"].center = x, y

            # -> Update scan vision cone position and orientation
            self.team_members[robot_id]["vision_cone_artist"].set_center((x, y))

            if self.team_members[robot_id]["pose"]["w"] < 0:
                theta_1 = 360 + self.team_members[robot_id]["pose"]["w"] - self.team_members[robot_id]["collision_cone_angle"]/2

                theta_2 = 360 + self.team_members[robot_id]["pose"]["w"]+ self.team_members[robot_id]["collision_cone_angle"]/2

            else:
                theta_1 = self.team_members[robot_id]["pose"]["w"] - self.team_members[robot_id]["collision_cone_angle"]/2

                theta_2 = self.team_members[robot_id]["pose"]["w"] + self.team_members[robot_id]["collision_cone_angle"]/2

            self.team_members[robot_id]["vision_cone_artist"].set_theta1(theta_1)
            self.team_members[robot_id]["vision_cone_artist"].set_theta2(theta_2)

            # -> Update scan vision cone color based on collision detected
            if self.team_members[robot_id]["on_collision_course"]:
                self.team_members[robot_id]["vision_cone_artist"].fc = (1,0,0,0.5)
            else:
                self.team_members[robot_id]["vision_cone_artist"].fc = (0,1,0,0.5)


        # -> Blit updated artists
        self.bm.update()

    def remove_robot(self, robot_id):
        try:
            # -> Delete subscribers
            self.node.destroy_subscription(self.team_members[robot_id]["pose_subscriber"])
            self.node.destroy_subscription(self.team_members[robot_id]["lazer_scan_subscriber"])

            # -> Remove artists from blit manager
            self.bm.remove_artist(self.team_members[robot_id]["pose_artist"])
            self.bm.remove_artist(self.team_members[robot_id]["goal_artist"])
            self.bm.remove_artist(self.team_members[robot_id]["goal_ray_artist"])
            self.bm.remove_artist(self.team_members[robot_id]["label_artist"])
            self.bm.remove_artist(self.team_members[robot_id]["scan_circle_artist"])
            self.bm.remove_artist(self.team_members[robot_id]["vision_cone_artist"])

            # -> Remove widget
            for i in reversed(range(self.ui.fleet_overview_layout.count())): 
                if self.ui.fleet_overview_layout.itemAt(i).widget().robot_name.text() == robot_id:
                    self.ui.fleet_overview_layout.itemAt(i).widget().deleteLater()
                    break

            # -> Delete member entry
            del self.team_members[robot_id]

        except:
            pass

    def add_robot(self, msg):
        lazer_radius = 0.5
        collision_cone_angle = 80

        # -> Create entry in team members dictionary
        (pose_artist,) = self.sc.axes.plot([], [], 'bo')
        (goal_artist,) = self.sc.axes.plot([], [], '-o', linewidth=0.1)
        (goal_ray_artist,) = self.sc.axes.plot([0, 0], [0, 0], linestyle='dashed', linewidth=0.5, color='blue')

        self.team_members[msg.robot_id] = {
            # ---------------------------------------- Base setup
            "overview_widget": Member_overview_widget(),
            "label_artist": self.sc.axes.annotate(
                xy=(0,0),
                xytext=(1, 1),
                s=msg.robot_id
                ),   

            # ---------------------------------------- Pose setup
            "pose_subscriber": None,
            "pose": {
                "x": 0,
                "y": 0,
                "z": 0,
                "u": 0,
                "v": 0,
                "w": 0
                },
            "pose_artist": pose_artist,

            # ---------------------------------------- Goal setup
            "goal": {
                "id": "",
                "current_subgoal": 0,
                "x": [],
                "y": []
                },
            "goal_artist": goal_artist,
            "goal_ray_artist": goal_ray_artist,      

            # ---------------------------------------- Lazer scan setup
            "lazer_scan_subscriber": None,
            "lazer_scan": None,
            "scan_circle_artist": mpatches.Circle(
                (0, 0),
                lazer_radius,
                fill=False,
                linestyle="--",
                linewidth=0.1
                ),

            "lazer_radius": lazer_radius,
            "collision_cone_angle": collision_cone_angle, 
            "on_collision_course": False,
            "vision_cone_artist": mpatches.Wedge(
                (0, 0), 
                lazer_radius, 
                -collision_cone_angle/2, 
                collision_cone_angle/2,
                fc=(0,1,0,0.5))
        }

        # -> Add patches to axes
        self.sc.axes.add_patch(self.team_members[msg.robot_id]["scan_circle_artist"])
        self.sc.axes.add_patch(self.team_members[msg.robot_id]["vision_cone_artist"])

        # -> Add artist to blit
        self.bm.add_artist(pose_artist)
        self.bm.add_artist(goal_artist)
        self.bm.add_artist(goal_ray_artist)

        self.bm.add_artist(self.team_members[msg.robot_id]["scan_circle_artist"])
        self.bm.add_artist(self.team_members[msg.robot_id]["vision_cone_artist"])
        self.bm.add_artist(self.team_members[msg.robot_id]["label_artist"])

        # -> Update member widget
        self.team_members[msg.robot_id]["overview_widget"].ui.robot_name.setText(msg.robot_id)
        self.team_members[msg.robot_id]["overview_widget"].ui.robot_name.setStyleSheet("font-weight: bold")

        self.team_members[msg.robot_id]["overview_widget"].ui.goal_id.setText("None")

        # -> Connect buttons
        self.team_members[msg.robot_id]["overview_widget"].ui.remove_robot.clicked.connect(partial(self.remove_robot, msg.robot_id))

        # -> Add member overview widget to main ui
        self.ui.fleet_overview_layout.addWidget(self.team_members[msg.robot_id]["overview_widget"].ui)

        # -> Create pose subscribers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.team_members[msg.robot_id]["pose_subscriber"] = self.node.create_subscription(
            msg_type=PoseStamped,
            topic=f"/{msg.robot_id}/pose",
            callback=partial(self.pose_subscriber_callback, msg.robot_id),
            qos_profile=qos
            )

        # -> Create lazer scan subscriber
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.team_members[msg.robot_id]["lazer_scan_subscriber"] = self.node.create_subscription(
            msg_type=LaserScan,
            topic=f"/{msg.robot_id}/scan",
            callback=partial(self.lazer_scan_subscriber_callback, msg.robot_id),
            qos_profile=qos
        )

    @staticmethod
    def __euler_from_quaternion(quat):
        """
        Convert quaternion (w in last place) to euler roll, pitch, yaw (rad).
        quat = [x, y, z, w]

        """
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp) * 180 / math.pi

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp) * 180 / math.pi

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp) * 180 / math.pi

        return roll, pitch, yaw


class MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parents, width=5, height=4, dpi=100):
        self.axes = None
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.built_canvas()
        super(MplCanvas, self).__init__(self.fig)

        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

    def built_canvas(self):
        x_min = -2
        x_max = 2

        y_min = -2
        y_max = 2

        self.axes = self.fig.add_subplot(111)
        # self.axes.axis("off")
        self.axes.set_xlim(x_min, x_max)
        self.axes.set_ylim(y_min, y_max)

        # -> Move left y-axis and bottim x-axis to centre, passing through (0,0)
        self.axes.spines['left'].set_position('center')
        self.axes.spines['bottom'].set_position('center')

        # -> Eliminate upper and right axes
        self.axes.spines['right'].set_color('none')
        self.axes.spines['top'].set_color('none')

        self.fig.tight_layout(pad=0)

        self.axes.grid()

        # -> Set aspect ratio to 1
        ratio = 1.0
        self.axes.set_aspect(abs((x_min-x_max)/(y_min-y_max))*ratio)

def main(args=None):
    # `rclpy` library is initialized
    rclpy.init(args=args)

    RLB_viz_gui()


if __name__ == '__main__':
    main()
