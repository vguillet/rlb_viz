

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

# Own modules
from .Blit_manager import BlitManager

# from rlb_controller.robot_parameters import *

##################################################################################################################



class Room_view:
    def __init__(self) -> None:
        # -> Create canvas widgets
        self.room_plot = MplCanvas(self,
                                   width=2,
                                   height=2,
                                   dpi=100)

        # -> Create blit managers
        self.room_bm = BlitManager(canvas=self.room_plot.fig.canvas)

        # -> Add plots to views
        self.ui.main_layout_room.addWidget(self.room_plot)

        # -> Create canvas tools widget
        toolbar = NavigationToolbar(self.room_plot, self.ui)
        self.ui.main_layout_room.addWidget(toolbar)
               
    def room_plot_robots(self):
        for robot_id in self.team_members.keys():
            x = round(self.team_members[robot_id]["pose"]["x"], 3)
            y = round(self.team_members[robot_id]["pose"]["y"], 3)

            # -> Update pose
            self.team_members[robot_id]["room_pose_artist"].set_xdata(x)
            self.team_members[robot_id]["room_pose_artist"].set_ydata(y)

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
            from rlb_config.robot_parameters import vision_cones, side_vision_cones

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
            from rlb_config.robot_parameters import collsion_ray_length

            if self.team_members[robot_id]["pose"]["w"] < 0:
                w = 360 + self.team_members[robot_id]["pose"]["w"]
            else:
                w = self.team_members[robot_id]["pose"]["w"]

            x_end = collsion_ray_length * math.cos(w*math.pi/180)
            y_end = collsion_ray_length * math.sin(w*math.pi/180)

            self.team_members[robot_id]["room_direction_pointer_artist"].set_xdata([x, x + x_end])
            self.team_members[robot_id]["room_direction_pointer_artist"].set_ydata([y, y + y_end])

        # -> Blit updated artists
        self.room_bm.update()

    def room_remove_robot(self, robot_id):
        from rlb_config.robot_parameters import vision_cones, side_vision_cones

        try:
            # -> Remove artists from blit manager
            self.room_bm.remove_artist(self.team_members[robot_id]["room_pose_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["goal_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["goal_ray_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["label_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["scan_circle_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["lazer_scan_point_cloud"])
            self.room_bm.remove_artist(self.team_members[robot_id]["collision_circle_artist"])
            self.room_bm.remove_artist(self.team_members[robot_id]["room_direction_pointer_artist"])

            for cone_ref in vision_cones.keys():
                self.room_bm.remove_artist(self.team_members[robot_id][cone_ref]["vision_cone_artist"])            
                
            for cone_ref in side_vision_cones.keys():
                self.room_bm.remove_artist(self.team_members[robot_id][cone_ref]["l_vision_cone_artist"])
                self.room_bm.remove_artist(self.team_members[robot_id][cone_ref]["r_vision_cone_artist"])

        except:
            pass

    def room_add_robot(self, msg):
        from rlb_config.robot_parameters import vision_cones, side_vision_cones, safety_zone

        # -> Add a patch for every vision cone
        for cone_ref, cone_properties in vision_cones.items():
            self.team_members[msg.source][cone_ref] = {
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
            self.room_plot.axes.add_patch(self.team_members[msg.source][cone_ref]["vision_cone_artist"])

            # -> Add artist to blit
            self.room_bm.add_artist(self.team_members[msg.source][cone_ref]["vision_cone_artist"])

        for cone_ref, cone_properties in side_vision_cones.items():
            self.team_members[msg.source][cone_ref] = {
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
            self.team_members[msg.source][cone_ref]["l_vision_cone_artist"].set(visible=False)
            self.team_members[msg.source][cone_ref]["r_vision_cone_artist"].set(visible=False)

            # -> Add artist to plot
            self.room_plot.axes.add_patch(self.team_members[msg.source][cone_ref]["l_vision_cone_artist"])
            self.room_plot.axes.add_patch(self.team_members[msg.source][cone_ref]["r_vision_cone_artist"])

            # -> Add artist to blit
            self.room_bm.add_artist(self.team_members[msg.source][cone_ref]["l_vision_cone_artist"])
            self.room_bm.add_artist(self.team_members[msg.source][cone_ref]["r_vision_cone_artist"])

        # ---------------------------------------- Base setup
        (lazer_scan_point_cloud_artist, ) = self.room_plot.axes.plot([], [], '.', markersize=1, color="black")
        (goal_ray_artist,) = self.room_plot.axes.plot([0, 0], [0, 0], linestyle='dashed', linewidth=1., color='blue')
        (goal_artist,) = self.room_plot.axes.plot([], [], '-o', linewidth=0.1)
        (room_direction_pointer_artist, ) = self.room_plot.axes.plot([0, 0], [0, 0], linewidth=.5, color='green')
        (room_pose_artist,) = self.room_plot.axes.plot([], [], 'co')

        self.team_members[msg.source]["label_artist"] = self.room_plot.axes.annotate(
            xy=(0,0),
            xytext=(1, 1),
            text=msg.source
            )

        # ---------------------------------------- Pose setup
        self.team_members[msg.source]["room_pose_artist"] = room_pose_artist
        self.team_members[msg.source]["room_direction_pointer_artist"] = room_direction_pointer_artist

        # ---------------------------------------- Goal setup
        self.team_members[msg.source]["goal_artist"] = goal_artist
        self.team_members[msg.source]["goal_ray_artist"] = goal_ray_artist  

        # ---------------------------------------- Lazer scan setup
        self.team_members[msg.source]["lazer_scan_point_cloud"] = lazer_scan_point_cloud_artist
        self.team_members[msg.source]["scan_circle_artist"] = mpatches.Circle(
            (0, 0),
            self.ui.lazer_scan_slider.value()/10,
            fill=False,
            linestyle="--",
            linewidth=0.1
            )

        self.team_members[msg.source]["collision_circle_artist"] = mpatches.Circle(
            (0, 0),
            0.1,
            fill=False,
            color="black",
            linewidth=safety_zone, 
            zorder=98
            )

        # -> Add patches to axes
        self.room_plot.axes.add_patch(self.team_members[msg.source]["scan_circle_artist"])
        self.room_plot.axes.add_patch(self.team_members[msg.source]["collision_circle_artist"])

        # -> Add artists to blit
        # Room
        self.room_bm.add_artist(room_pose_artist)
        self.room_bm.add_artist(goal_artist)
        self.room_bm.add_artist(goal_ray_artist)
        self.room_bm.add_artist(lazer_scan_point_cloud_artist)
        self.room_bm.add_artist(room_direction_pointer_artist)

        self.room_bm.add_artist(self.team_members[msg.source]["scan_circle_artist"])
        self.room_bm.add_artist(self.team_members[msg.source]["collision_circle_artist"])
        self.room_bm.add_artist(self.team_members[msg.source]["label_artist"])

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
