

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
        self.map_img = image.imread(sim_map_image_path)

        # -> Add map to plot
        aspect_ratio = self.map_img.shape[1]/self.map_img.shape[0]

        if aspect_ratio < 1:
            self.sim_map_plot.axes.imshow(
                self.map_img,
                extent=(-3*aspect_ratio, 3*aspect_ratio, -3, 3)
                )

            self.sim_map_plot.axes.set_xlim(3*aspect_ratio, -3*aspect_ratio)
        
        else:
            self.sim_map_plot.axes.imshow(
                self.map_img,
                extent=(-3, 3, -3*aspect_ratio, 3*aspect_ratio)
                )

            self.sim_map_plot.axes.set_ylim(3*aspect_ratio, -3*aspect_ratio) 

        # -> Format coordinates
        def format_coord(x, y):
            lat, lon = self.convert_coordinates(x, y)
            return 'Long: {:6.6f}, Lat: {:6.6f}'.format(lat, lon)

        self.sim_map_plot.axes.format_coord = format_coord

        # -> Create blit managers
        self.sim_map_bm = BlitManager(canvas=self.sim_map_plot.fig.canvas)

        # -> Add plots to views
        self.ui.main_layout_sim_map.addWidget(self.sim_map_plot)

        # -> Create canvas tools widget
        toolbar = NavigationToolbar(self.sim_map_plot, self.ui)
        self.ui.main_layout_sim_map.addWidget(toolbar)

    def convert_coordinates(self, x, y):
        # TOFIX: Conversion function
        # -> Setting reference points
        ref_1_pix =(7, 193)
        ref_1_geo = (44.27303, 1.72456)

        ref_2_pix = (871, 908)
        ref_2_geo = (44.276598, 1.730598)

        # -> Calculating differences
        ref_dx_pix = abs(ref_2_pix[0] - ref_1_pix[0])
        ref_dy_pix = abs(ref_2_pix[1] - ref_1_pix[1])

        dlat = abs(ref_2_geo[0] - ref_1_geo[0])
        dlon = abs(ref_2_geo[1] - ref_1_geo[1])

        # -> Solving for per pixel lat/lon shift
        dx_geo_shift = dlat/ref_dx_pix
        dy_geo_shift = dlon/ref_dy_pix

        # -> Convert provided coordinates to image reference frame
        dx_img = self.map_img.shape[0]
        dy_img = self.map_img.shape[1]

        dx_canvas = abs(self.sim_map_plot.axes.get_xlim()[1]) + abs(self.sim_map_plot.axes.get_xlim()[0])
        dy_canvas = abs(self.sim_map_plot.axes.get_ylim()[1]) + abs(self.sim_map_plot.axes.get_ylim()[0])

        dx_img_shift = dx_img/dx_canvas
        dy_img_shift = dy_img/dy_canvas

        # -> Convert provided coordinates to geo reference frame
        ref_1_canvas = (ref_1_pix[0] * dx_img_shift, ref_1_pix[1] * dy_img_shift)
    
        # x
        dx = abs(ref_1_canvas[0] - x)

        if x < ref_1_canvas[0]:
            x_lat =  ref_1_geo[0] - dx*dx_img_shift*dx_geo_shift

        else:
            x_lat =  ref_1_geo[0] + dx*dx_img_shift*dx_geo_shift

        # y
        dy = abs(ref_1_canvas[1] - y)

        if y < ref_1_canvas[1]:
            y_lon =  ref_1_geo[1] - dy*dy_img_shift*dy_geo_shift

        else:
            y_lon =  ref_1_geo[1] + dy*dy_img_shift*dy_geo_shift

        return x_lat, y_lon

    def sim_map_plot_robots(self):
        for robot_id in self.team_members.keys():
            x = round(self.team_members[robot_id]["pose"]["x"], 3)
            y = round(self.team_members[robot_id]["pose"]["y"], 3)

            # -> Update coordinated collision ray
            from rlb_controller.robot_parameters import collsion_ray_length

            if self.team_members[robot_id]["pose"]["w"] < 0:
                w = 360 + self.team_members[robot_id]["pose"]["w"]
            else:
                w = self.team_members[robot_id]["pose"]["w"]

            x_end = collsion_ray_length * math.cos(w*math.pi/180)
            y_end = collsion_ray_length * math.sin(w*math.pi/180)

            self.team_members[robot_id]["sim_map_direction_pointer_artist"].set_xdata([x, x + x_end])
            self.team_members[robot_id]["sim_map_direction_pointer_artist"].set_ydata([y, y + y_end])

            # -> Update pose
            self.team_members[robot_id]["sim_map_pose_artist"].set_xdata(x)
            self.team_members[robot_id]["sim_map_pose_artist"].set_ydata(y)

        # -> Blit updated artists
        self.sim_map_bm.update()

    def sim_map_remove_robot(self, robot_id):
        try:
            # -> Remove artists from blit manager
            self.sim_map_bm.remove_artist(self.team_members[robot_id]["sim_map_pose_artist"])
            self.sim_map_bm.remove_artist(self.team_members[robot_id]["sim_map_direction_pointer_artist"])

        except:
            pass

    def sim_map_add_robot(self, msg):
        (sim_map_direction_pointer_artist, ) = self.sim_map_plot.axes.plot([0, 0], [0, 0], linewidth=.5, color='green')
        (sim_map_pose_artist,) = self.sim_map_plot.axes.plot([], [], 'co')

        # ---------------------------------------- Pose setup
        self.team_members[msg.robot_id]["sim_map_direction_pointer_artist"] = sim_map_direction_pointer_artist
        self.team_members[msg.robot_id]["sim_map_pose_artist"] = sim_map_pose_artist

        # -> Add artists to blit
        self.sim_map_bm.add_artist(sim_map_pose_artist)
        self.sim_map_bm.add_artist(sim_map_direction_pointer_artist)

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
