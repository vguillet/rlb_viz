

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
import matplotlib.image as image
from PyQt5.QtCore import *
import numpy as np

# Own modules
from .Blit_manager import BlitManager

# from rlb_controller.robot_parameters import *

##################################################################################################################



class Sim_map_view:
    def __init__(self) -> None:
        # -> Create canvas widgets
        self.sim_map_plot = MplCanvas(self,
                                   width=2,
                                   height=2,
                                   dpi=100)

        # -> Load maps
        from rlb_controller.simulation_parameters import sim_map_image_path
        map_img = image.imread(sim_map_image_path)

        # -> Add map to plot
        aspect_ratio = map_img.shape[1]/map_img.shape[0]

        if aspect_ratio < 1:
            self.sim_map_plot.axes.imshow(
                map_img,
                extent=(-3*aspect_ratio, 3*aspect_ratio, -3, 3)
                )

            self.sim_map_plot.axes.set_xlim(3*aspect_ratio, -3*aspect_ratio)
        
        else:
            self.sim_map_plot.axes.imshow(
                map_img,
                extent=(-3, 3, -3*aspect_ratio, 3*aspect_ratio)
                )

            self.sim_map_plot.axes.set_ylim(3*aspect_ratio, -3*aspect_ratio) 


        # -> Create blit managers
        self.sim_map_bm = BlitManager(canvas=self.sim_map_plot.fig.canvas)

        # -> Add plots to views
        self.ui.main_layout_sim_map.addWidget(self.sim_map_plot)

        # -> Create canvas tools widget
        toolbar = NavigationToolbar(self.sim_map_plot, self.ui)
        self.ui.main_layout_sim_map.addWidget(toolbar)
               
    def sim_map_plot_robots(self):
        for robot_id in self.team_members.keys():
            x = round(self.team_members[robot_id]["pose"]["x"], 3)
            y = round(self.team_members[robot_id]["pose"]["y"], 3)

            # -> Update pose
            self.team_members[robot_id]["sim_map_pose_artist"].set_xdata(x)
            self.team_members[robot_id]["sim_map_pose_artist"].set_ydata(y)

        # -> Blit updated artists
        self.sim_map_bm.update()

    def sim_map_remove_robot(self, robot_id):
        try:
            # -> Remove artists from blit manager
            self.sim_map_bm.remove_artist(self.team_members[robot_id]["sim_map_pose_artist"])

        except:
            pass

    def sim_map_add_robot(self, msg):
        (sim_map_pose_artist,) = self.sim_map_plot.axes.plot([], [], 'bo')

        # ---------------------------------------- Pose setup
        self.team_members[msg.robot_id]["sim_map_pose_artist"] = sim_map_pose_artist

        # -> Add artists to blit
        self.sim_map_bm.add_artist(sim_map_pose_artist)

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
