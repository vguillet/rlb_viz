

##################################################################################################################
"""
"""

# Built-in/Generic Imports
import math

# Libs
import math
import matplotlib.patches as mpatches
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from PyQt5.QtCore import *
import numpy as np

# Own modules
from .Blit_manager import BlitManager

# from rlb_controller.robot_parameters import *

##################################################################################################################



class Room_energy_surface_view:
    def __init__(self) -> None:
        # -> Create canvas widgets
        self.room_energy_surface_plot = MplCanvas(self,
                                   width=2,
                                   height=2,
                                   dpi=100)

        # -> Create blit managers
        self.room_energy_surface_bm = BlitManager(canvas=self.room_energy_surface_plot.fig.canvas)

        # -> Add plots to views
        self.ui.main_layout_room_energy_surface.addWidget(self.room_energy_surface_plot)

        # -> Create canvas tools widget
        toolbar = NavigationToolbar(self.room_energy_surface_plot, self.ui)
        self.ui.main_layout_room_energy_surface.addWidget(toolbar)
               
    def room_energy_surface_plot_robots(self):
        # for robot_id in self.team_members.keys():
        #     x = round(self.team_members[robot_id]["pose"]["x"], 3)
        #     y = round(self.team_members[robot_id]["pose"]["y"], 3)

        #     # -> Update pose
        #     self.team_members[robot_id]["room_energy_surface_pose_artist"].set_xdata(x)
        #     self.team_members[robot_id]["room_energy_surface_pose_artist"].set_ydata(y)

        # -> Blit updated artists
        self.room_energy_surface_bm.update()

    def room_energy_surface_remove_robot(self, robot_id):
        from rlb_controller.robot_parameters import vision_cones, side_vision_cones

        try:
            # -> Remove artists from blit manager
            self.room_energy_surface_bm.remove_artist(self.team_members[robot_id]["room_energy_surface_pose_artist"])

        except:
            pass

    def room_energy_surface_add_robot(self, msg):
        from rlb_controller.robot_parameters import vision_cones, side_vision_cones

        (room_energy_surface_pose_artist,) = self.room_energy_surface_plot.axes.plot([], [], [], 'bo')

        X = np.arange(-3, 3, 0.25)
        Y = np.arange(-3, 3, 0.25)
        X, Y = np.meshgrid(X, Y)
        R = np.sqrt(X ** 2 + Y ** 2)
        Z = np.sin(R)

        room_energy_surface_artist = self.room_energy_surface_plot.axes.plot_surface(X, Y, Z, cmap="plasma")


        # ---------------------------------------- Pose setup
        self.team_members[msg.robot_id]["room_energy_surface_pose_artist"] = room_energy_surface_pose_artist
        self.team_members[msg.robot_id]["room_energy_surface_artist"] = room_energy_surface_artist

        # -> Add artists to blit
        self.room_energy_surface_bm.add_artist(room_energy_surface_pose_artist)
        self.room_energy_surface_bm.add_artist(room_energy_surface_artist)


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

        self.axes = self.fig.add_subplot(111, projection="3d")
        # self.axes.axis("off")
        self.axes.set_xlim(x_min, x_max)
        self.axes.set_ylim(y_min, y_max)
        self.axes.set_zlim(y_min, y_max)

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
