

##################################################################################################################
"""
"""

# Built-in/Generic Imports
from cProfile import label
import math
import copy

# Libs
import math
import matplotlib.patches as mpatches
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from PyQt5.QtCore import *
import numpy as np

# Own modules
from .Blit_manager import BlitManager
from rlb_coordinator.Caylus_map_loader import load_maps
from rlb_coordinator.Raster_ray_tracing import check_comms_available

# from rlb_controller.robot_parameters import *
from rlb_controller.simulation_parameters import *

##################################################################################################################



class Sim_comms_view:
    def __init__(self) -> None:
        # -> Create comms integrity monitor plot
        self.sim_comms_integrity_monitor_plot = Simple_MplCanvas(self,
                                                                 width=2,
                                                                 height=2,
                                                                 dpi=100)

        # -> Add plot to views
        self.ui.comms_integrity_monitor.addWidget(self.sim_comms_integrity_monitor_plot)

        # -> Create blit manager
        self.comms_integrity_monitor_bm = BlitManager(canvas=self.sim_comms_integrity_monitor_plot.fig.canvas)

        # -> Create canvas widgets
        self.sim_comms_plot = MplCanvas(self,
                                        width=2,
                                        height=2,
                                        dpi=100)

        # -> Load obstacle_grids
        obstacle_grids = load_maps(
            hard_obstacles=True,
            dense_vegetation=True,
            light_vegetation=True,
            paths=False
        )

        # -> Generate signal blocking probability grid
        self.signal_blocking_prob_grid = \
                obstacle_grids["hard_obstacles"] * hard_obstacles_signal_blocking_prob \
                + obstacle_grids["dense_vegetation"] * dense_vegetation_signal_blocking_prob \
                + obstacle_grids["light_vegetation"] * light_vegetation_signal_blocking_prob

        # -> Clip min/max values
        self.signal_blocking_prob_grid = np.clip(self.signal_blocking_prob_grid, 0, 1)

        # -> Add map to plot
        aspect_ratio = self.signal_blocking_prob_grid.shape[1]/self.signal_blocking_prob_grid.shape[0]

        if aspect_ratio < 1:
            self.sim_comms_plot.axes.imshow(
                self.signal_blocking_prob_grid,
                extent=(-3*aspect_ratio, 3*aspect_ratio, -3, 3),
                cmap='plasma'
                )

            self.sim_comms_plot.axes.set_xlim(-3*aspect_ratio, 3*aspect_ratio)
        
        else:
            self.sim_comms_plot.axes.imshow(
                self.signal_blocking_prob_grid,
                extent=(-3, 3, -3*aspect_ratio, 3*aspect_ratio),
                cmap='plasma'
                )

            self.sim_comms_plot.axes.set_ylim(-3*aspect_ratio, 3*aspect_ratio) 

        # -> Invert axis
        self.sim_comms_plot.axes.invert_xaxis()
        self.sim_comms_plot.axes.invert_yaxis()

        # -> Format coordinates
        def format_coord(x, y):
            (x_pix, y_pix) = self.convert_coords_room_to_pixel((x, y), self.sim_comms_plot.axes)
            return "x (pixels): {:6.1f}, y (pixels): {:6.1f}".format(x_pix, y_pix)

        self.sim_comms_plot.axes.format_coord = format_coord

        # -> Create blit manager
        self.sim_comms_bm = BlitManager(canvas=self.sim_comms_plot.fig.canvas)

        # -> Add plot to views
        self.ui.main_layout_sim_comms.addWidget(self.sim_comms_plot)

        # -> Create canvas tools widget
        toolbar = NavigationToolbar(self.sim_comms_plot, self.ui)
        self.ui.main_layout_sim_comms.addWidget(toolbar)

    def sim_comms_plot_robots(self):
        # -> Update robots pairs
        for agent_pair in self.agent_pairs:
            x1 = round(self.team_members[agent_pair[0]]["pose"]["x"], 3)
            y1 = round(self.team_members[agent_pair[0]]["pose"]["y"], 3)

            x2 = round(self.team_members[agent_pair[1]]["pose"]["x"], 3)
            y2 = round(self.team_members[agent_pair[1]]["pose"]["y"], 3)

            # -> Check comms state
            point_1_pix = self.convert_coords_room_to_pixel(point_room=(x1, y1), plot_axes=self.sim_comms_plot.axes)
            point_2_pix = self.convert_coords_room_to_pixel(point_room=(x2, y2), plot_axes=self.sim_comms_plot.axes)

            self.comm_rays[agent_pair]["comm_state"], comms_integrity_profile, ray_coordinates = check_comms_available(
                pose_1=point_1_pix,
                pose_2=point_2_pix,
                obstacle_probabilities_grid=self.signal_blocking_prob_grid)

            # -> Update comm integrity profile
            self.comm_rays[agent_pair]["comms_integrity_artist"].set_xdata(np.arange(0, len(comms_integrity_profile)))
            self.comm_rays[agent_pair]["comms_integrity_artist"].set_ydata([1-number for number in comms_integrity_profile])

            # -> Update comms ray
            if self.comm_rays[agent_pair]["comm_state"]:
                self.comm_rays[agent_pair]["sim_comms_comm_ray_artist"].set(visible=True)

                self.comm_rays[agent_pair]["sim_comms_comm_ray_artist"].set_xdata([x1, x2])
                self.comm_rays[agent_pair]["sim_comms_comm_ray_artist"].set_ydata([y1, y2])  
                
            else:
                self.comm_rays[agent_pair]["sim_comms_comm_ray_artist"].set(visible=False)
        
        # -> Update robots
        for robot_id in self.team_members.keys():
            x = round(self.team_members[robot_id]["pose"]["x"], 3)
            y = round(self.team_members[robot_id]["pose"]["y"], 3)

            # -> Update direction ray
            from rlb_controller.robot_parameters import collsion_ray_length

            if self.team_members[robot_id]["pose"]["w"] < 0:
                w = 360 + self.team_members[robot_id]["pose"]["w"]
            else:
                w = self.team_members[robot_id]["pose"]["w"]

            x_end = collsion_ray_length * math.cos(w*math.pi/180)
            y_end = collsion_ray_length * math.sin(w*math.pi/180)

            self.team_members[robot_id]["sim_comms_direction_pointer_artist"].set_xdata([x, x + x_end])
            self.team_members[robot_id]["sim_comms_direction_pointer_artist"].set_ydata([y, y + y_end])

            # -> Update pose
            self.team_members[robot_id]["sim_comms_pose_artist"].set_xdata(x)
            self.team_members[robot_id]["sim_comms_pose_artist"].set_ydata(y)

        # -> Blit updated artists
        self.comms_integrity_monitor_bm.update()
        self.sim_comms_bm.update()

    def sim_comms_remove_robot(self, robot_id):
        try:
            # -> Remove artists from blit manager
            self.sim_comms_bm.remove_artist(self.team_members[robot_id]["sim_comms_pose_artist"])
            self.sim_comms_bm.remove_artist(self.team_members[robot_id]["sim_comms_direction_pointer_artist"])

            for agent_pair in self.agent_pairs:
                if robot_id in agent_pair:
                    self.sim_comms_bm.remove_artist(self.comm_rays[agent_pair]["sim_comms_comm_ray_artist"])
                    self.comms_integrity_monitor_bm.remove_artist(self.comm_rays[agent_pair]["comms_integrity_artist"])
                    
                    del self.comm_rays[agent_pair]["sim_comms_comm_ray_artist"]
                    del self.comm_rays[agent_pair]["comms_integrity_artist"]
            
        except:
            pass

    def sim_comms_add_robot(self, msg):
        # -> Add comm_ray for every agent pair
        for agent_pair in self.agent_pairs:
            if "sim_comms_comm_ray_artist" not in self.comm_rays[agent_pair].keys():
                # -> Create ray artist
                (comm_ray_artist,) = self.sim_comms_plot.axes.plot([], [], linestyle='dashed', linewidth=1., color='white', zorder=0)
                self.comm_rays[agent_pair]["sim_comms_comm_ray_artist"] = comm_ray_artist

                # -> Add artist to blit
                self.sim_comms_bm.add_artist(comm_ray_artist)
            
            if "comms_integrity_artist" not in self.comm_rays[agent_pair].keys():
                # -> Create ray artist
                (comm_integrity_artist,) = self.sim_comms_integrity_monitor_plot.axes.plot([], [], linewidth=1., label=str(agent_pair))
                self.comm_rays[agent_pair]["comms_integrity_artist"] = comm_integrity_artist

                # -> Add artist to blit
                self.comms_integrity_monitor_bm.add_artist(comm_integrity_artist)

        (sim_comms_direction_pointer_artist, ) = self.sim_comms_plot.axes.plot([], [], linewidth=.5, color='green')
        (sim_comms_pose_artist,) = self.sim_comms_plot.axes.plot([], [], 'co')

        # ---------------------------------------- Pose setup
        self.team_members[msg.robot_id]["sim_comms_direction_pointer_artist"] = sim_comms_direction_pointer_artist
        self.team_members[msg.robot_id]["sim_comms_pose_artist"] = sim_comms_pose_artist

        # -> Add artists to blit
        self.sim_comms_bm.add_artist(sim_comms_direction_pointer_artist)
        self.sim_comms_bm.add_artist(sim_comms_pose_artist)


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

        self.fig.tight_layout(pad=0)

        self.axes.grid()

        # -> Set aspect ratio to 1
        ratio = 1.0
        self.axes.set_aspect(abs((x_min-x_max)/(y_min-y_max))*ratio)


class Simple_MplCanvas(FigureCanvasQTAgg):
    def __init__(self, parents, width=5, height=4, dpi=100):
        self.axes = None
        self.fig = Figure(figsize=(width, height), dpi=dpi)
        self.built_canvas()
        super(Simple_MplCanvas, self).__init__(self.fig)

        self.setFocusPolicy(Qt.ClickFocus)
        self.setFocus()

    def built_canvas(self):
        x_min = 0
        x_max = 500

        y_min = 0
        y_max = 1.2

        self.axes = self.fig.add_subplot(111)
        # self.axes.axis("off")
        self.axes.set_xlim(x_min, x_max)
        self.axes.set_ylim(y_min, y_max)

        self.fig.tight_layout(pad=0)

        self.axes.grid()

        # -> Set aspect ratio to 1
        ratio = 0.3
        self.axes.set_aspect(abs((x_min-x_max)/(y_min-y_max))*ratio)
