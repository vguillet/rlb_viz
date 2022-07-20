

##################################################################################################################
"""
"""

# Built-in/Generic Imports
import json
import os
import queue
import math

# Libs
import PyQt5
from PyQt5.QtCore import *
import sys
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import JointState, LaserScan
from rlb_utils.msg import Goal, TeamComm
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math

import matplotlib.patches as mpatches
from matplotlib.collections import PatchCollection
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.image as image

from functools import partial

# Own modules
# from rlb_coordinator.Caylus_map_loader import load_maps
from .UI_singletons import Ui_singleton
from .Member_overview_widget import Member_overview_widget
from .Blit_manager import BlitManager
from networkx.readwrite import json_graph
import networkx as nx


# from rlb_controller.robot_parameters import *

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
        # -> Load maps
        from rlb_controller.simulation_parameters import sim_map_image_path
        map_img = image.imread(sim_map_image_path)

        # grids_lst = load_maps(
        #     hard_obstacles=True,
        #     dense_vegetation=True,
        #     light_vegetation=True,
        #     paths=True
        # )

        # -> Create canvas widgets
        # ----- Room
        self.room_plot = MplCanvas(self,
                                   width=2,
                                   height=2,
                                   dpi=100)

        # ----- Map
        self.sim_map_plot = MplCanvas(self,
                                      width=2,
                                      height=2,
                                      dpi=100)

        # -> Add map to plot
        aspect_ratio = map_img.shape[1]/map_img.shape[0]

        if aspect_ratio < 1:
            self.sim_map_plot.axes.imshow(
                map_img,
                extent=(-3*aspect_ratio, 3*aspect_ratio, -3, 3)
                )

            self.sim_map_plot.axes.set_xlim(-3*aspect_ratio, 3*aspect_ratio)
        
        else:
            self.sim_map_plot.axes.imshow(
                map_img,
                extent=(-3, 3, -3*aspect_ratio, 3*aspect_ratio)
                )

            self.sim_map_plot.axes.set_ylim(-3*aspect_ratio, 3*aspect_ratio) 

        # ----- Paths
        self.sim_paths_plot = MplCanvas(self,
                                        width=2,
                                        height=2,
                                        dpi=100)
        # ----- Comms
        self.sim_comms_plot = MplCanvas(self,
                                        width=2,
                                        height=2,
                                        dpi=100)

        # # -> Load map
        # path = str(os.getcwd()) + "/ros2_ws/src/rlb_viz/rlb_viz/Graphs/data.json"
        # f = open (path, "r")
        # data = json.loads(f.read())

        # G = nx.Graph([("A", "B")])
        # data = json_graph.node_link_data(G)

        # self.map = json_graph.node_link_graph(data)

        # print(self.map)

        # subax1 = plt.subplot(121)
        # nx.draw(G, with_labels=True, font_weight='bold')
        # subax2 = plt.subplot(122)
        # nx.draw_shell(G, nlist=[range(5, 10), range(5)], with_labels=True, font_weight='bold')

        # -> Create blit managers
        self.room_bm = BlitManager(canvas=self.room_plot.fig.canvas)
        self.sim_map_bm = BlitManager(canvas=self.sim_map_plot.fig.canvas)
        self.sim_paths_bm = BlitManager(canvas=self.sim_paths_plot.fig.canvas)
        self.sim_comms_bm = BlitManager(canvas=self.sim_comms_plot.fig.canvas)

        # -> Add plots to views
        self.ui.main_layout_room.addWidget(self.room_plot)
        self.ui.main_layout_simulation_map.addWidget(self.sim_map_plot)
        self.ui.main_layout_simulation_paths.addWidget(self.sim_paths_plot)
        self.ui.main_layout_simulation_comm.addWidget(self.sim_comms_plot)

        # -> Create canvas tools widget
        toolbar = NavigationToolbar(self.room_plot, self.ui)
        self.ui.main_layout_room.addWidget(toolbar)

        toolbar = NavigationToolbar(self.sim_map_plot, self.ui)
        self.ui.main_layout_simulation_map.addWidget(toolbar)

        toolbar = NavigationToolbar(self.sim_paths_plot, self.ui)
        self.ui.main_layout_simulation_paths.addWidget(toolbar)

        toolbar = NavigationToolbar(self.sim_comms_plot, self.ui)
        self.ui.main_layout_simulation_comm.addWidget(toolbar)

        # -> Display windows
        self.ui.show()

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
        
        # ==================================================== Create publishers


        # ==================================================== Create subscribers
        # ----------------------------------- Team communications publisher
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        self.team_comms_publisher = self.node.create_publisher(
            msg_type=TeamComm,
            topic="/Team_comms",
            qos_profile=qos
            )

        # ----------------------------------- Team communications subscriber
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_ALL,
            )

        self.team_comms_subscriber = self.node.create_subscription(
            msg_type=TeamComm,
            topic="/Team_comms",
            callback=self.team_msg_subscriber_callback,
            qos_profile=qos
            )
        
        # -> To remove
        self.placeholder_callback()

        # self.team_comms_msgs = queue.Queue()

        # ----------------------------------- Goal subscribers
        from rlb_controller.robot_parameters import goals_topic

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
        for robot_id in self.team_members.keys():
            x = round(self.team_members[robot_id]["pose"]["x"], 3)
            y = round(self.team_members[robot_id]["pose"]["y"], 3)

            # -> Update pose
            self.team_members[robot_id]["room_pose_artist"].set_xdata(x)
            self.team_members[robot_id]["room_pose_artist"].set_ydata(y)

            self.team_members[robot_id]["sim_map_pose_artist"].set_xdata(x)
            self.team_members[robot_id]["sim_map_pose_artist"].set_ydata(y)

            self.team_members[robot_id]["sim_paths_pose_artist"].set_xdata(x)
            self.team_members[robot_id]["sim_paths_pose_artist"].set_ydata(y)

            self.team_members[robot_id]["sim_comms_pose_artist"].set_xdata(x)
            self.team_members[robot_id]["sim_comms_pose_artist"].set_ydata(y)

            # -------------------------- Update goal
            if self.ui.goals_view_toggle.isChecked():
                self.team_members[robot_id]["goal_artist"].set(visible=True)
                self.team_members[robot_id]["goal_ray_artist"].set(visible=True)

                goal_x = self.team_members[robot_id]["goal"]["x"]
                goal_y = self.team_members[robot_id]["goal"]["y"]

                self.team_members[robot_id]["goal_artist"].set_data(
                    goal_x,
                    goal_y
                    )

                # -> Update goal ray
                if goal_x:
                    self.team_members[robot_id]["goal_ray_artist"].set_xdata([x, goal_x[0]])

                    self.team_members[robot_id]["goal_ray_artist"].set_ydata([y, goal_y[0]])

                else:
                    self.team_members[robot_id]["goal_ray_artist"].set_xdata([])
                    self.team_members[robot_id]["goal_ray_artist"].set_ydata([])

            else:
                self.team_members[robot_id]["goal_artist"].set(visible=False)
                self.team_members[robot_id]["goal_ray_artist"].set(visible=False)


            # -> Update annoation position
            self.team_members[robot_id]["label_artist"].xy = (x, y)
            self.team_members[robot_id]["label_artist"].set_position((x+0.05, y+0.1))

            # -------------------------- Update vision cones
            from rlb_controller.robot_parameters import vision_cones, side_vision_cones

            if self.ui.vision_cones_toggle.isChecked():
                # -> Update frontal cones
                for cone_ref in vision_cones.keys():
                    self.team_members[robot_id][cone_ref]["vision_cone_artist"].set(visible=True)

                    # -> Update scan vision cone position and orientation
                    self.team_members[robot_id][cone_ref]["vision_cone_artist"].set_center((x, y))

                    if self.team_members[robot_id]["pose"]["w"] < 0:
                        theta_1 = 360 + self.team_members[robot_id]["pose"]["w"] - self.team_members[robot_id][cone_ref]["angle"]/2
                        theta_2 = 360 + self.team_members[robot_id]["pose"]["w"] + self.team_members[robot_id][cone_ref]["angle"]/2

                    else:
                        theta_1 = self.team_members[robot_id]["pose"]["w"] - self.team_members[robot_id][cone_ref]["angle"]/2
                        theta_2 = self.team_members[robot_id]["pose"]["w"] + self.team_members[robot_id][cone_ref]["angle"]/2

                    self.team_members[robot_id][cone_ref]["vision_cone_artist"].set_theta1(theta_1)
                    self.team_members[robot_id][cone_ref]["vision_cone_artist"].set_theta2(theta_2)

                    # -> Update scan vision cone color based on collision detected
                    if self.team_members[robot_id][cone_ref]["triggered"]:
                        self.team_members[robot_id][cone_ref]["vision_cone_artist"].set(fc=(1,0,0,0.5))

                    else:
                        self.team_members[robot_id][cone_ref]["vision_cone_artist"].set(fc=(0,1,0,0.5))

                # -> Update side cones
                for cone_ref in side_vision_cones.keys():
                    # -> Update scan vision cone position and orientation
                    self.team_members[robot_id][cone_ref]["l_vision_cone_artist"].set_center((x, y))
                    self.team_members[robot_id][cone_ref]["r_vision_cone_artist"].set_center((x, y))

                    if self.team_members[robot_id]["pose"]["w"] < 0:
                        l_theta_1 = 90 + 360 + self.team_members[robot_id]["pose"]["w"] - self.team_members[robot_id][cone_ref]["angle"]/2
                        l_theta_2 = 90 + 360 + self.team_members[robot_id]["pose"]["w"] + self.team_members[robot_id][cone_ref]["angle"]/2

                        r_theta_1 = -90 + 360 + self.team_members[robot_id]["pose"]["w"] - self.team_members[robot_id][cone_ref]["angle"]/2
                        r_theta_2 = -90 + 360 + self.team_members[robot_id]["pose"]["w"] + self.team_members[robot_id][cone_ref]["angle"]/2

                    else:
                        l_theta_1 = 90 + self.team_members[robot_id]["pose"]["w"] - self.team_members[robot_id][cone_ref]["angle"]/2
                        l_theta_2 = 90 + self.team_members[robot_id]["pose"]["w"] + self.team_members[robot_id][cone_ref]["angle"]/2

                        r_theta_1 = -90 + self.team_members[robot_id]["pose"]["w"] - self.team_members[robot_id][cone_ref]["angle"]/2
                        r_theta_2 = -90 + self.team_members[robot_id]["pose"]["w"] + self.team_members[robot_id][cone_ref]["angle"]/2

                    self.team_members[robot_id][cone_ref]["l_vision_cone_artist"].set_theta1(l_theta_1)
                    self.team_members[robot_id][cone_ref]["l_vision_cone_artist"].set_theta2(l_theta_2)

                    self.team_members[robot_id][cone_ref]["r_vision_cone_artist"].set_theta1(r_theta_1)
                    self.team_members[robot_id][cone_ref]["r_vision_cone_artist"].set_theta2(r_theta_2)

                    # -> Update scan vision cone color based on collision detected
                    if self.team_members[robot_id][cone_ref]["l_triggered"]:
                        
                        self.team_members[robot_id][cone_ref]["l_vision_cone_artist"].set(fc=(1,0,0,0.5))

                    else:
                        self.team_members[robot_id][cone_ref]["l_vision_cone_artist"].set(fc=(0,1,0,0.5))

                    if self.team_members[robot_id][cone_ref]["r_triggered"]:
                        self.team_members[robot_id][cone_ref]["r_vision_cone_artist"].set(fc=(1,0,0,0.5))

                    else:
                        self.team_members[robot_id][cone_ref]["r_vision_cone_artist"].set(fc=(0,1,0,0.5))
            
            else:
                # -> Update frontal cones
                for cone_ref in vision_cones.keys():
                    self.team_members[robot_id][cone_ref]["vision_cone_artist"].set(visible=False)

                # -> Update side cones
                for cone_ref in side_vision_cones.keys():
                    self.team_members[robot_id][cone_ref]["l_vision_cone_artist"].set(visible=False)
                    self.team_members[robot_id][cone_ref]["r_vision_cone_artist"].set(visible=False)
            
            # -------------------------- Update other collision artists
            # -> Update collision circle position
            self.team_members[robot_id]["collision_circle_artist"].center = x, y

            # -> Update coordinated collision ray
            from rlb_controller.robot_parameters import collsion_ray_length

            if self.team_members[robot_id]["pose"]["w"] < 0:
                w = 360 + self.team_members[robot_id]["pose"]["w"]
            else:
                w = self.team_members[robot_id]["pose"]["w"]

            x_end = collsion_ray_length * math.cos(w*math.pi/180)
            y_end = collsion_ray_length * math.sin(w*math.pi/180)

            self.team_members[robot_id]["coordinated_collision_ray_artist"].set_xdata([x, x + x_end])
            self.team_members[robot_id]["coordinated_collision_ray_artist"].set_ydata([y, y + y_end])

        # -> Blit updated artists
        self.room_bm.update()
        self.sim_map_bm.update()
        self.sim_paths_bm.update()
        self.sim_comms_bm.update()

    def remove_robot(self, robot_id):
        from rlb_controller.robot_parameters import vision_cones, side_vision_cones

        try:
            # -> Delete subscribers
            self.node.destroy_subscription(self.team_members[robot_id]["pose_subscriber"])
            self.node.destroy_subscription(self.team_members[robot_id]["lazer_scan_subscriber"])

            # -> Remove artists from blit manager
            self.room_bm.remove_artist(self.team_members[robot_id]["room_pose_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["goal_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["goal_ray_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["label_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["scan_circle_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["lazer_scan_point_cloud"])
            self.room_bm.remove_artist(self.team_members[robot_id]["collision_circle_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["coordinated_collision_ray_artist"])

            for cone_ref in vision_cones.keys():
                self.room_bm.remove_artist(self.team_members[robot_id][cone_ref]["vision_cone_artist"])            
                
            for cone_ref in side_vision_cones.keys():
                self.room_bm.remove_artist(self.team_members[robot_id][cone_ref]["l_vision_cone_artist"])
                self.room_bm.remove_artist(self.team_members[robot_id][cone_ref]["r_vision_cone_artist"])

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
        from rlb_controller.robot_parameters import vision_cones, side_vision_cones

        # -> Create entry in team members dictionary
        (room_pose_artist,) = self.room_plot.axes.plot([], [], 'bo')
        (sim_map_pose_artist,) = self.sim_map_plot.axes.plot([], [], 'bo')
        (sim_paths_pose_artist,) = self.sim_paths_plot.axes.plot([], [], 'bo')
        (sim_comms_pose_artist,) = self.sim_comms_plot.axes.plot([], [], 'bo')

        (goal_artist,) = self.room_plot.axes.plot([], [], '-o', linewidth=0.1)
        (goal_ray_artist,) = self.room_plot.axes.plot([0, 0], [0, 0], linestyle='dashed', linewidth=1., color='blue')
        (lazer_scan_point_cloud_artist, ) = self.room_plot.axes.plot([], [], '.', markersize=1, color="black")
        (coordinated_collision_ray_artist, ) =self.room_plot.axes.plot([0, 0], [0, 0], linewidth=.5, color='green')

        self.team_members[msg.robot_id] = {
            # ---------------------------------------- Base setup
            "overview_widget": Member_overview_widget(),
            "label_artist": self.room_plot.axes.annotate(
                xy=(0,0),
                xytext=(1, 1),
                text=msg.robot_id
                ),   
            "state": None,

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
            "room_pose_artist": room_pose_artist,
            "sim_map_pose_artist": sim_map_pose_artist,
            "sim_paths_pose_artist": sim_paths_pose_artist,
            "sim_comms_pose_artist": sim_comms_pose_artist,

            # ---------------------------------------- Goal setup
            "goal": {
                "id": "",
                "x": [],
                "y": []
                },
            "goal_artist": goal_artist,
            "goal_ray_artist": goal_ray_artist,      

            # ---------------------------------------- Lazer scan setup
            "lazer_scan_subscriber": None,
            "lazer_scan": None,
            "lazer_scan_point_cloud": lazer_scan_point_cloud_artist,
            "scan_circle_artist": mpatches.Circle(
                (0, 0),
                self.ui.lazer_scan_slider.value()/10,
                fill=False,
                linestyle="--",
                linewidth=0.1
                ),
            "collision_circle_artist": mpatches.Circle(
                (0, 0),
                0.10,
                fill=False,
                color="black",
                linewidth=1
                ),
            # ---------------------------------------- Coordinated collision setup
            "coordinated_collision_ray_artist": coordinated_collision_ray_artist,
        }

        # -> Add patches to axes
        self.room_plot.axes.add_patch(self.team_members[msg.robot_id]["scan_circle_artist"])
        self.room_plot.axes.add_patch(self.team_members[msg.robot_id]["collision_circle_artist"])

        # -> Add artists to blit
        # Room
        self.room_bm.add_artist(room_pose_artist)
        self.room_bm.add_artist(goal_artist)
        self.room_bm.add_artist(goal_ray_artist)
        self.room_bm.add_artist(lazer_scan_point_cloud_artist)
        self.room_bm.add_artist(coordinated_collision_ray_artist)

        self.room_bm.add_artist(self.team_members[msg.robot_id]["scan_circle_artist"])
        self.room_bm.add_artist(self.team_members[msg.robot_id]["collision_circle_artist"])
        self.room_bm.add_artist(self.team_members[msg.robot_id]["label_artist"])

        # -> Add a patch for every vision cone
        for cone_ref, cone_properties in vision_cones.items():
            self.team_members[msg.robot_id][cone_ref] = {
                "treshold": cone_properties["threshold"],
                "angle": cone_properties["angle"],
                "triggered": False,
                "vision_cone_artist": mpatches.Wedge(
                    (0, 0), 
                    cone_properties["threshold"], 
                    -cone_properties["angle"]/2, 
                    cone_properties["angle"]/2,
                    fc=(0,1,0,0.5)),
            }

            # -> Add artist to plot
            self.room_plot.axes.add_patch(self.team_members[msg.robot_id][cone_ref]["vision_cone_artist"])

            # -> Add artist to blit
            self.room_bm.add_artist(self.team_members[msg.robot_id][cone_ref]["vision_cone_artist"])

        for cone_ref, cone_properties in side_vision_cones.items():
            self.team_members[msg.robot_id][cone_ref] = {
                "treshold": cone_properties["threshold"],
                "angle": cone_properties["angle"],
                "l_triggered": False,
                "r_triggered": False,
                "l_vision_cone_artist": mpatches.Wedge(
                    (0, 0), 
                    cone_properties["threshold"], 
                    90 - cone_properties["angle"]/2, 
                    90 + cone_properties["angle"]/2,
                    fc=(0,1,0,0.5)),
                "r_vision_cone_artist": mpatches.Wedge(
                    (0, 0), 
                    cone_properties["threshold"], 
                    -90 - cone_properties["angle"]/2, 
                    -90 + cone_properties["angle"]/2,
                    fc=(0,1,0,0.5)),
            }

            # -> Hide side collision patches
            self.team_members[msg.robot_id][cone_ref]["l_vision_cone_artist"].set(visible=False)
            self.team_members[msg.robot_id][cone_ref]["r_vision_cone_artist"].set(visible=False)

            # -> Add artist to plot
            self.room_plot.axes.add_patch(self.team_members[msg.robot_id][cone_ref]["l_vision_cone_artist"])
            self.room_plot.axes.add_patch(self.team_members[msg.robot_id][cone_ref]["r_vision_cone_artist"])

            # -> Add artist to blit
            self.room_bm.add_artist(self.team_members[msg.robot_id][cone_ref]["l_vision_cone_artist"])
            self.room_bm.add_artist(self.team_members[msg.robot_id][cone_ref]["r_vision_cone_artist"])

        # Map
        self.sim_map_bm.add_artist(sim_map_pose_artist)
        # self.sim_map_bm.add_artist(self.team_members[msg.robot_id]["collision_circle_artist"])
        # self.sim_map_bm.add_artist(self.team_members[msg.robot_id]["label_artist"])

        # Paths
        self.sim_paths_bm.add_artist(sim_paths_pose_artist)
        # self.sim_paths_bm.add_artist(self.team_members[msg.robot_id]["collision_circle_artist"])
        # self.sim_paths_bm.add_artist(self.team_members[msg.robot_id]["label_artist"])

        # Comms
        self.sim_comms_bm.add_artist(sim_comms_pose_artist)
        # self.sim_comms_bm.add_artist(self.team_members[msg.robot_id]["collision_circle_artist"])
        # self.sim_comms_bm.add_artist(self.team_members[msg.robot_id]["label_artist"])

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


        x_min = -3
        x_max = 3

        y_min = -3
        y_max = 3

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