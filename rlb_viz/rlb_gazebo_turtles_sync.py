

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
from PyQt5.QtGui import QIcon

from functools import partial

import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, Point, Pose
from sensor_msgs.msg import JointState, LaserScan
from gazebo_msgs.msg import ModelState, EntityState
from gazebo_msgs.srv import SetModelState, SetEntityState
from rlb_utils.msg import Goal, TeamComm
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
import math

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar

##################################################################################################################

"""
ros2 service call /gazebo/set_model_state gazebo_msgs/srv/SetModelState '{model_state: { model_name: turtlebot3_burger, pose: { position: { x: 0.3, y: 0.2 ,z: 0 }, orientation: {x: 0, y: 0.491983115673, z: 0, w: 0.870604813099 } }, twist: { linear: : {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
"""

class Rlb_gazebo_turtles_sync:
    def rlb_gazebo_remove_robot(self, robot_id):
        self.node.destroy_publisher(self.team_members[robot_id]["rlb_gazebo_state_publisher"])
        self.node.destroy_timer(self.team_members[robot_id]["rlb_gazebo_state_publisher_timer"])

    def rlb_gazebo_add_robot(self, msg):

        self.team_members[msg.source]["rlb_gazebo_model_state_service"] = self.node.create_client(
            SetEntityState,
            "/gazebo/set_entity_state"
        )

        timer_period = 0.001  # seconds
        self.team_members[msg.source]["rlb_gazebo_model_state_service_timer"] = self.node.create_timer(
            timer_period, 
            partial(self.gazebo_model_pose_service_request, msg.source),
        )

        # # -> Setup agent rlb_gazebo_state_publisher
        # qos = QoSProfile(depth=10)
        # self.team_members[msg.source]["rlb_gazebo_state_publisher"] = self.node.create_publisher(
        #     msg_type=EntityState,
        #     topic="/gazebo/set_entity_state",
        #     qos_profile=qos
        # )

        # # -> Setup publisher_timer
        # timer_period = 1.  # seconds

        # self.team_members[msg.source]["rlb_gazebo_state_publisher_timer"] = self.node.create_timer(
        #     timer_period, 
        #     partial(self.gazebo_publisher_callback, msg.source),
        # )

    def gazebo_model_pose_service_request(self,robot_id):
        if robot_id != "Turtle_1":
            return
            
        # -> Construct message
        req = SetEntityState.Request()
        req.state.name = 'turtlebot3_burger'

        # Pose
        req.state.pose.position.x = self.team_members[robot_id]["pose"]["x"]
        req.state.pose.position.y = self.team_members[robot_id]["pose"]["y"]
        req.state.pose.position.z = self.team_members[robot_id]["pose"]["z"] + 0.01

        # Orientation
        qx, qy, qz, qw = self.get_quaternion_from_euler(
            roll=self.team_members[robot_id]["pose"]["u"] * math.pi/180,
            pitch=self.team_members[robot_id]["pose"]["v"] * math.pi/180,
            yaw=self.team_members[robot_id]["pose"]["w"] * math.pi/180
        )

        req.state.pose.orientation.x = qx
        req.state.pose.orientation.y = qy
        req.state.pose.orientation.z = qz
        req.state.pose.orientation.w = qw

        # Twist
        req.state.twist.linear.x = 0.
        req.state.twist.linear.y = 0.
        req.state.twist.linear.z = 0.

        req.state.twist.angular.x = 0.
        req.state.twist.angular.y = 0.
        req.state.twist.angular.z = 0.

        # -> Publish message to gazebo topic
        self.team_members[robot_id]["rlb_gazebo_model_state_service"].call_async(req)

    # def gazebo_publisher_callback(self, robot_id):
    #     # -> Construct message
    #     msg = EntityState(name='turtlebot3_burger')

    #     # Pose
    #     factor = 10



    #     msg.pose.position.x = self.team_members[robot_id]["pose"]["x"] * factor
    #     msg.pose.position.y = self.team_members[robot_id]["pose"]["y"] * factor
    #     msg.pose.position.z = self.team_members[robot_id]["pose"]["z"] * factor

    #     # Orientation
    #     qx, qy, qz, qw = self.get_quaternion_from_euler(
    #         roll=self.team_members[robot_id]["pose"]["u"],
    #         pitch=self.team_members[robot_id]["pose"]["v"],
    #         yaw=self.team_members[robot_id]["pose"]["w"]
    #     )

    #     msg.pose.orientation.x = qx
    #     msg.pose.orientation.y = qy
    #     msg.pose.orientation.z = qz
    #     msg.pose.orientation.w = qw

    #     # Twist
    #     msg.twist.linear.x = 0.
    #     msg.twist.linear.y = 0.
    #     msg.twist.linear.z = 0.

    #     msg.twist.angular.x = 0.
    #     msg.twist.angular.y = 0.
    #     msg.twist.angular.z = 0.

    #     # -> Publish message to gazebo topic
    #     self.team_members[robot_id]["rlb_gazebo_state_publisher"].publish(msg=msg)

    @staticmethod
    def get_quaternion_from_euler(roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        
        Input
            :param roll: The roll (rotation around x-axis) angle in radians.
            :param pitch: The pitch (rotation around y-axis) angle in radians.
            :param yaw: The yaw (rotation around z-axis) angle in radians.
        
        Output
            :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
        """
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        return qx, qy, qz, qw