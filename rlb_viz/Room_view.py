

class Room_view:       
    def plot_robot(self):
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

    def add_robot(self, robot_dict: dict, msg):
        from rlb_controller.robot_parameters import vision_cones, side_vision_cones

        (room_pose_artist,) = self.room_plot.axes.plot([], [], 'bo')
        (goal_artist,) = self.room_plot.axes.plot([], [], '-o', linewidth=0.1)
        (goal_ray_artist,) = self.room_plot.axes.plot([0, 0], [0, 0], linestyle='dashed', linewidth=1., color='blue')
        (lazer_scan_point_cloud_artist, ) = self.room_plot.axes.plot([], [], '.', markersize=1, color="black")
        (coordinated_collision_ray_artist, ) = self.room_plot.axes.plot([0, 0], [0, 0], linewidth=.5, color='green')

        self.team_members[msg.robot_id] = {
            # ---------------------------------------- Base setup
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

            # ---------------------------------------- Goal setup
            "goal": {
                "id": "",
                "x": [],
                "y": []
                },
            "goal_artist": goal_artist,
            "goal_ray_artist": goal_ray_artist,      

            # ---------------------------------------- Lazer scan setup
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