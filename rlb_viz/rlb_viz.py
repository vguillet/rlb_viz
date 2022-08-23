

##################################################################################################################
"""
"""

# Built-in/Generic Imports
import json
import os
import queue
import math
from functools import partial

# Libs
import PyQt5
from PyQt5.QtCore import *
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.QtGui import QIcon

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import JointState, LaserScan
from rlb_utils.msg import Goal, TeamComm
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np

import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

# Own modules
from .Room_view import Room_view
from .Room_energy_surface_view import Room_energy_surface_view
from .Sim_map_view import Sim_map_view
from .Sim_comms_view import Sim_comms_view
from .Sim_paths_view import Sim_paths_view
from .rlb_gazebo_turtles_sync import Rlb_gazebo_turtles_sync

# from rlb_coordinator.Caylus_map_loader import load_maps
from .UI_singletons import Ui_singleton
from .Member_overview_widget import Member_overview_widget
from .Blit_manager import BlitManager
from networkx.readwrite import json_graph
import networkx as nx


# from rlb_controller.robot_parameters import *

##################################################################################################################


class RLB_viz_gui(
    Room_view,
    Room_energy_surface_view,
    Sim_map_view,
    Sim_comms_view,
    Sim_paths_view,
    Rlb_gazebo_turtles_sync
    ):
    def __init__(self):
        # ==================================================== Load GUI
        app = QtWidgets.QApplication([])

        # -> Load gui singleton
        self.ui = Ui_singleton().interface

        root_path = str(os.getcwd())
        self.ui.setWindowIcon(QIcon(root_path + "/ros2_ws/src/rlb_viz/rlb_viz/Data/Turtlebot3_logo.png"))
        self.ui.setWindowTitle("rlb viz")

        # -> Connect buttons
        self.ui.add_robot.clicked.connect(self.__manual_add_robot)

        # ==================================================== Create visualiser
        # -> Create plotter timer
        self.plot_timer = QtCore.QTimer()
        self.plot_timer.timeout.connect(self.plot_robots)
        self.plot_timer.setInterval(10)
        self.plot_timer.start()
        
        # ==================================================== Create Node
        self.node = Node("RLB_viz")

        # -> QT timer based spin
        self.spin_timer = QtCore.QTimer()
        self.spin_timer.timeout.connect(self.__node_spinner)
        self.spin_timer.setInterval(1)
        self.spin_timer.start(0.1)

        # ==================================================== Setup storage variables
        self.team_members = {}
        self.comm_rays = {}
        
        # ==================================================== Create publishers
        # ----------------------------------- Team communications publisher
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        self.team_comms_publisher = self.node.create_publisher(
            msg_type=TeamComm,
            topic="/team_comms",
            qos_profile=qos
            )

        # ==================================================== Create subscribers
        # ----------------------------------- Team communications subscriber
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        self.team_comms_subscriber = self.node.create_subscription(
            msg_type=TeamComm,
            topic="/team_comms",
            callback=self.team_msg_subscriber_callback,
            qos_profile=qos
            )

        # ----------------------------------- Goal subscribers
        from rlb_config.robot_parameters import goals_topic

        qos = QoSProfile(depth=10)

        self.goal_subscription = self.node.create_subscription(
            msg_type=Goal,
            topic=goals_topic,
            # topic="/sim_node_publisher/rlb/targets",
            callback=self.goal_subscriber_callback,
            qos_profile=qos
            )

        # self.goals_msgs = queue.Queue()

        # ==================================================== Final setup
        # -> Load IHM modules
        Room_view.__init__(self)
        Room_energy_surface_view.__init__(self)
        Sim_map_view.__init__(self)
        Sim_comms_view.__init__(self)
        Sim_paths_view.__init__(self)

        # -> To remove
        self.placeholder_callback()

        # -> Display windows
        self.ui.show()

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
        if msg.robot_id not in self.team_members.keys() and self.ui.auto_add_robots.isChecked():
            self.add_robot(msg=msg)

        if msg.type == "Goal_annoucement":
            self.__set_goal(msg=msg)

        elif msg.type == "Collision":
            from rlb_controller.robot_parameters import vision_cones, side_vision_cones

            msg_content = json.loads(msg.memo)

            cone_ref = msg_content["cone_triggered"]

            if msg_content["collision_state"] in [1, 2]:
                self.__clear_triggers(msg=msg)

                # -> Flag triggered cone
                if cone_ref in vision_cones.keys():
                    self.team_members[msg.robot_id][cone_ref]["triggered"] = True
                else:
                    self.team_members[msg.robot_id][cone_ref][msg_content["side"] + "_triggered"] = True  

                # -> Make relevant patch collection visible
                for side_cone_ref in side_vision_cones.keys():
                    self.team_members[msg.robot_id][side_cone_ref][msg_content["side"] + "_vision_cone_artist"].set(visible=True)
            
            elif msg_content["collision_state"] == 0:
                self.__clear_triggers(msg=msg)

        # -> Add msg to team_comms text area
        self.ui.team_comms_msgs.setPlainText(str(msg))

    def __clear_triggers(self, msg):
        from rlb_controller.robot_parameters import vision_cones, side_vision_cones

        for cone_ref in vision_cones.keys():
            self.team_members[msg.robot_id][cone_ref]["triggered"] = False

        for cone_ref in side_vision_cones.keys():
            self.team_members[msg.robot_id][cone_ref]["l_triggered"] = False
            self.team_members[msg.robot_id][cone_ref]["r_triggered"] = False

            # -> Hide side collision patches
            self.team_members[msg.robot_id][cone_ref]["l_vision_cone_artist"].set(visible=False)
            self.team_members[msg.robot_id][cone_ref]["r_vision_cone_artist"].set(visible=False)


    def goal_subscriber_callback(self, msg):
        # -> Add msg to goals text area
        self.ui.goals_msgs.setPlainText(str(msg))

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

    def pose_projected_subscriber_callback(self, robot_id, msg):
        # -> Update position
        self.team_members[robot_id]["pose_projected"]["x"] = msg.pose.position.x
        self.team_members[robot_id]["pose_projected"]["y"] = msg.pose.position.y
        self.team_members[robot_id]["pose_projected"]["z"] = msg.pose.position.z

        # -> Update orientation
        u, v, w = self.__euler_from_quaternion(quat=msg.pose.orientation)

        self.team_members[robot_id]["pose_projected"]["u"] = u
        self.team_members[robot_id]["pose_projected"]["v"] = v
        self.team_members[robot_id]["pose_projected"]["w"] = w

    def lazer_scan_subscriber_callback(self, robot_id, msg):
        scan = list(msg.ranges)
        
        range_min = msg.range_min
        range_max = msg.range_max

        # -> Clean up ranges
        for i, range_measure in enumerate(scan):
            if range_measure < range_min or range_measure > range_max:
                scan[i] = None

        self.team_members[robot_id]["lazer_scan"] = scan
        
        # -> Update lidar point cloud
        if self.ui.lidar_view_toggle.isChecked():
            self.team_members[robot_id]["scan_circle_artist"].set(visible=True)
            self.team_members[robot_id]["lazer_scan_point_cloud"].set(visible=True)

            # -> Update scan circle position
            x = round(self.team_members[robot_id]["pose"]["x"], 3)
            y = round(self.team_members[robot_id]["pose"]["y"], 3)

            self.team_members[robot_id]["scan_circle_artist"].center = x, y
            self.team_members[robot_id]["scan_circle_artist"].set_radius(self.ui.lazer_scan_slider.value()/10)

            # -> Update lazer scan point cloud
            points_x_list, points_y_list = self.__get_lazer_scan_point_cloud(robot_id=robot_id)

            self.team_members[robot_id]["lazer_scan_point_cloud"].set_data(
                points_x_list,
                points_y_list
                )

        else:
            self.team_members[robot_id]["scan_circle_artist"].set(visible=False)
            self.team_members[robot_id]["lazer_scan_point_cloud"].set(visible=False)

    # ================================================= Utils
    def convert_coords_room_to_pixel(self, point_room, plot_axes):
        from rlb_config.simulation_parameters import images_shape

        # -> Calculating differences
        dx_img = images_shape[0]
        dy_img = images_shape[1]

        dx_canvas = abs(plot_axes.get_xlim()[0]) + abs(plot_axes.get_xlim()[1])
        dy_canvas = abs(plot_axes.get_ylim()[0]) + abs(plot_axes.get_ylim()[1])

        # -> Solving for scaling factor
        dx_img_shift = dx_img/dx_canvas
        dy_img_shift = dy_img/dy_canvas

        return (int(point_room[0] * dx_img_shift + dx_img/2), int(point_room[1] * dy_img_shift + dy_img/2))
    
    def convert_coords_pixel_to_latlon(self, point_pixel):
        from rlb_config.simulation_parameters import ref_1_pixel, ref_1_latlon
        from rlb_config.simulation_parameters import ref_2_pixel, ref_2_latlon

        # -> Calculating differences
        ref_dx_pixel = abs(ref_2_pixel[1] - ref_1_pixel[1])
        ref_dy_pixel = abs(ref_2_pixel[0] - ref_1_pixel[0])

        ref_dlon = abs(ref_2_latlon[0] - ref_1_latlon[0])
        ref_dlat = abs(ref_2_latlon[1] - ref_1_latlon[1])

        # -> Solving for scaling factor
        dx_lon_shift = ref_dlon/ref_dx_pixel
        dy_lat_shift = ref_dlat/ref_dy_pixel

        # -> Solving for origin latlon
        origin_lon = ref_1_latlon[0] - ref_1_pixel[1] * dx_lon_shift
        origin_lat = ref_1_latlon[1] - ref_1_pixel[0] * dy_lat_shift

        return (origin_lon + point_pixel[1] * dx_lon_shift, 
                origin_lat + point_pixel[0] * dy_lat_shift)
    
    def convert_coords_room_to_latlon(self, point_room, plot_axes):
        point_pixel = self.convert_coords_room_to_pixel(point_room=point_room, plot_axes=plot_axes)
        point_latlon = self.convert_coords_pixel_to_latlon(point_pixel=point_pixel)

        return point_latlon

    @property
    def agent_pairs(self):
        agent_list = list(self.team_members.keys())
        agent_pairs = [(a, b) for idx, a in enumerate(agent_list) for b in agent_list[idx + 1:]]

        return agent_pairs

    def __get_lazer_scan_point_cloud(self, robot_id):
        point_cloud_x = []
        point_cloud_y = []

        # -> Get orientation and pose
        pose_x = self.team_members[robot_id]["pose"]["x"]
        pose_y = self.team_members[robot_id]["pose"]["y"]
        orientation = self.team_members[robot_id]["pose"]["w"]

        # -> Correct coordinate system
        if orientation < 0:
            orientation = (180 - abs(orientation)) + 180       

        orientation -= 90

        if orientation < 0:
            orientation += 360

        orientation = 360 - orientation
    
        for i, point_distance in enumerate(self.team_members[robot_id]["lazer_scan"]):
            if point_distance is not None:
                if point_distance < self.ui.lazer_scan_slider.value()/10:
                    point_cloud_x.append(point_distance * math.sin((orientation - i) * math.pi/180) + pose_x)
                    point_cloud_y.append(point_distance * math.cos((orientation - i) * math.pi/180) + pose_y)

        return point_cloud_x, point_cloud_y

    def __set_goal(self, msg):
        # -> Save goal to member's dict
        goal = json.loads(msg.memo)

        self.team_members[msg.robot_id]["goal"]["id"] = goal["id"]

        sequence_xs = []
        sequence_ys = []

        for point in goal["sequence"]:
            sequence_xs.append(point[0])
            sequence_ys.append(point[1])

        self.team_members[msg.robot_id]["goal"]["x"] = sequence_xs
        self.team_members[msg.robot_id]["goal"]["y"] = sequence_ys

        # -> Set widget view
        self.team_members[msg.robot_id]["overview_widget"].ui.goal_id.setText(goal["id"])

        if sequence_xs and sequence_ys:
            self.team_members[msg.robot_id]["overview_widget"].ui.goal_x.setText(str(round(sequence_xs[0], 2)))
            self.team_members[msg.robot_id]["overview_widget"].ui.goal_y.setText(str(round(sequence_ys[0], 2)))
        else:
            self.team_members[msg.robot_id]["overview_widget"].ui.goal_x.setText("-")
            self.team_members[msg.robot_id]["overview_widget"].ui.goal_y.setText("-")

    def __manual_add_robot(self):
        msg = TeamComm()
        msg.robot_id = self.ui.add_robot_text_entry.text()
        msg.type = "Initial"
        msg.memo = ""

        # -> Add robot if not already added
        if msg.robot_id not in self.team_members.keys():
            self.add_robot(msg=msg)

    def plot_robots(self):
        # -> Plot views
        self.room_plot_robots()
        # self.room_energy_surface_plot_robots()
        self.sim_map_plot_robots()
        self.sim_comms_plot_robots()
        self.sim_paths_plot_robots()

    def remove_robot(self, robot_id):
        self.room_remove_robot(robot_id=robot_id)
        self.room_energy_surface_remove_robot(robot_id=robot_id)
        self.sim_map_remove_robot(robot_id=robot_id)
        self.sim_comms_remove_robot(robot_id=robot_id)
        self.sim_paths_remove_robot(robot_id=robot_id)
        self.rlb_gazebo_remove_robot(robot_id=robot_id)

        try:
            # -> Delete subscribers
            self.node.destroy_subscription(self.team_members[robot_id]["pose_subscriber"])
            self.node.destroy_subscription(self.team_members[robot_id]["lazer_scan_subscriber"])

            # -> Remove widget
            for i in reversed(range(self.ui.fleet_overview_layout_room.count())): 
                if self.ui.fleet_overview_layout_room.itemAt(i).widget().robot_name.text() == robot_id:
                    self.ui.fleet_overview_layout_room.itemAt(i).widget().deleteLater()
                    break

            # -> Delete member entry
            del self.team_members[robot_id]

        except:
            pass

    def add_robot(self, msg):
        # ---------------- Add team member entry to team members dict
        self.team_members[msg.robot_id] = {
            # -> Base setup
            "overview_widget": Member_overview_widget(), 
            "state": None,

            # -> Pose setup
            "pose_subscriber": None,
            "pose": {
                "x": 0.,
                "y": 0.,
                "z": 0.,
                "u": 0.,
                "v": 0.,
                "w": 0.
                },

            # -> Pose projected setup
            "pose_projected_subscriber": None,
            "pose_projected": {
                "x": 0.,
                "y": 0.,
                "z": 0.,
                "u": 0.,
                "v": 0.,
                "w": 0.
                },
                
            # -> Goal setup
            "goal": {
                "id": "",
                "x": [],
                "y": []
                },

            # -> Lazer scan setup
            "lazer_scan_subscriber": None,
            "lazer_scan": None,
        }
        # ---------------- Add team member entry to comm_rays
        for agent_pair in self.agent_pairs:
            if agent_pair not in self.comm_rays.keys():
                self.comm_rays[agent_pair] = {
                    "comm_state": True,
                    "comms_integrity_profile": []
                    }

        # -> Run modules add robot
        self.room_add_robot(msg = msg)
        self.room_energy_surface_add_robot(msg = msg)
        self.sim_map_add_robot(msg=msg)
        self.sim_comms_add_robot(msg=msg)
        self.sim_paths_add_robot(msg=msg)
        self.rlb_gazebo_add_robot(msg=msg)

        # -> Update member widget
        self.team_members[msg.robot_id]["overview_widget"].ui.robot_name.setText(msg.robot_id)
        self.team_members[msg.robot_id]["overview_widget"].ui.robot_name.setStyleSheet("font-weight: bold")

        self.team_members[msg.robot_id]["overview_widget"].ui.goal_id.setText("None")

        # -> Connect buttons
        self.team_members[msg.robot_id]["overview_widget"].ui.remove_robot.clicked.connect(partial(self.remove_robot, msg.robot_id))

        # -> Add member overview widget to main ui
        self.ui.fleet_overview_layout_room.addWidget(self.team_members[msg.robot_id]["overview_widget"].ui)

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

        # -> Create pose projected subscribers
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1
            )

        self.team_members[msg.robot_id]["pose_projected_subscriber"] = self.node.create_subscription(
            msg_type=PoseStamped,
            topic=f"/{msg.robot_id}/pose_projected",
            callback=partial(self.pose_projected_subscriber_callback, msg.robot_id),
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
        from rlb_controller.room_paramters import room_x_range, room_y_range
        x_min = room_x_range[0]
        x_max = room_x_range[1]

        y_min = room_y_range[0]
        y_max = room_y_range[1]

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

        # -> Invert axis
        self.axes.invert_xaxis()
        self.axes.invert_yaxis()

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