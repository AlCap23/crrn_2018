from simple_sim._simObjects import _SimObject

import pybullet


class Robot(_SimObject):
    """Base class for the robot model.
    """

    def __init__(self, server, urdf=None, **kwargs):
        super().__init__(server, urdf, **kwargs)

        # Add the robot to the server
        server.addRobot(self)

    def update(self):

        self._updateSliderValues()

        if hasattr(self, "controller"):
            for controller in self.controller.values():
                controller.update()

        if hasattr(self, "measurements"):
            self._collectJointMeasurements()
            self._collectLinkMeasurements()

        if hasattr(self, "_measurements"):
            self.server._measurements = self.server._measurements.append(
                self._measurements, ignore_index=True
            )

    def addController(self, controller):
        if not hasattr(self, "controller"):
            self.controller = {}

        uid = len(self.controller)
        self.controller.update({uid: controller})

    def addJointMeasurement(self, joint, variables=None):
        """ Measure a specific joint with specific variables ( Position, Velocity, Force) in generalized coordinates.
        """

        # Take only valid links
        if joint in self.joints.keys():
            joint_id = self.joints[joint]["jointIndex"]
        else:
            return

        variable_keys = ["position", "velocity", "force", "motor_torque"]

        variables = sorted(
            [variable for variable in variables if variable in variable_keys]
        )

        if not variables:
            return

        if not hasattr(self, "measurements"):
            self.measurements = {"joints": {}, "links": {}}

        if "forces" in variables:
            pybullet.enableJointForceTorqueSensor(
                bodyUniqueId=self.id,
                jointIndex=joint_id,
                enableSensor=True,
                physicsClientId=self.client_id,
            )

        self.measurements["joints"].update({joint: variables})

    def addLinkMeasurement(self, link, variables=None):
        """ Measure a specific joint with specific variables ( Position, Velocity, Force) in generalized coordinates.
        """

        # Take only valid links
        if not link in self.links.keys():
            return

        variable_keys = [
            "position",
            "orientation",
            "linear_velocity",
            "angular_velocity",
        ]

        frame = "global"

        variables = sorted(
            [variable for variable in variables if variable in variable_keys]
        )

        if not variables:
            return

        if not hasattr(self, "measurements"):
            self.measurements = {"joints": {}, "links": {}}

        self.measurements["links"].update(
            {link: {"variables": variables, "frame": frame}}
        )

    def _collectJointMeasurements(self):
        if self.measurements["joints"]:
            if not hasattr(self, "_measurements"):
                self._measurements = {}

            for joint, variables in (self.measurements["joints"]).items():
                joint_id = self.joints[joint]["jointIndex"]

                position, velocity, reaction_forces, motor_torque = pybullet.getJointState(
                    self.id, joint_id, self.client_id
                )

                if "force" in variables:
                    self._measurements.update({joint + "_force_x": reaction_forces[0]})
                    self._measurements.update({joint + "_force_y": reaction_forces[1]})
                    self._measurements.update({joint + "_force_z": reaction_forces[2]})
                    self._measurements.update({joint + "_torque_x": reaction_forces[3]})
                    self._measurements.update({joint + "_torque_y": reaction_forces[4]})
                    self._measurements.update({joint + "_torque_z": reaction_forces[5]})

                if "position" in variables:
                    self._measurements.update({joint + "_position": position})

                if "velocity" in variables:
                    self._measurements.update({joint + "_velocity": velocity})

                if "motor_torque" in variables:
                    self._measurements.update({joint + "_motor_torque": motor_torque})

    def _collectLinkMeasurements(self):
        if self.measurements["links"]:
            if not hasattr(self, "_measurements"):
                self._measurements = {}

            for link, measurement_info in (self.measurements["links"]).items():

                variables = measurement_info["variables"]
                frame = measurement_info["frame"]

                if link == "base":

                    pos_glo, orn_glo = pybullet.getBasePositionAndOrientation(
                        self.id, self.client_id
                    )

                    if (
                        "linear_velocity" in variables
                        or "angular_velocity" in variables
                    ):
                        vel_lin_glo, vel_ang_glo = pybullet.getBaseVelocity(
                            self.id, self.client_id
                        )

                else:

                    loadKeys = {
                        "bodyUniqueId": self.id,
                        "linkIndex": self._links[link],
                        "physicsClientId": self.client_id,
                    }

                    if (
                        "linear_velocity" in variables
                        or "angular_velocity" in variables
                    ):
                        loadKeys.update({"computeLinkVelocity": 1})
                        pos_glo, orn_glo, com_pos_loc, com_orn_loc, frame_pos_glo, frame_orn_glo, vel_lin_glo, vel_ang_glo = pybullet.getLinkState(
                            **loadKeys
                        )
                    else:
                        pos_glo, orn_glo, com_pos_loc, com_orn_loc, frame_pos_glo, frame_orn_glo = pybullet.getLinkState(
                            **loadKeys
                        )

                # Set orientation to euler
                orn_glo = pybullet.getEulerFromQuaternion(orn_glo)

                if frame is "global":
                    if "position" in variables:
                        self._measurements.update({link + "_position_x": pos_glo[0]})
                        self._measurements.update({link + "_position_y": pos_glo[1]})
                        self._measurements.update({link + "_position_z": pos_glo[2]})
                    if "linear_velocity" in variables:
                        self._measurements.update(
                            {link + "_linear_velocity_x": vel_lin_glo[0]}
                        )
                        self._measurements.update(
                            {link + "_linear_velocity_y": vel_lin_glo[1]}
                        )
                        self._measurements.update(
                            {link + "_linear_velocity_z": vel_lin_glo[2]}
                        )
                    if "orientation" in variables:
                        self._measurements.update({link + "_orientation_x": orn_glo[0]})
                        self._measurements.update({link + "_orientation_y": orn_glo[1]})
                        self._measurements.update({link + "_orientation_z": orn_glo[2]})
                    if "angular_velocity" in variables:
                        self._measurements.update(
                            {link + "_angular_velocity_x": vel_ang_glo[0]}
                        )
                        self._measurements.update(
                            {link + "_angular_velocity_y": vel_ang_glo[1]}
                        )
                        self._measurements.update(
                            {link + "_angular_velocity_z": vel_ang_glo[2]}
                        )


class StewartPlatform(Robot):
    def __init__(self, server, **kwargs):
        super().__init__(server, **kwargs)

        # Add the constraints
        parents = ["top_11", "top_12", "top_13"]
        children = {
            "top_11": ["ITF_31"],
            "top_12": ["ITF_22", "ITF_12"],
            "top_13": ["ITF_23", "ITF_33"],
        }

        self.constraints = {}

        for parent in parents:
            parent_id = self.joints[parent]["jointIndex"]
            for child in children[parent]:
                constraint_name = parent + "_2_" + child
                child_id = self.joints[child]["jointIndex"]
                # Create a p2p connection
                constraint_id = pybullet.createConstraint(
                    parentBodyUniqueId=self.id,
                    parentLinkIndex=parent_id,
                    childBodyUniqueId=self.id,
                    childLinkIndex=child_id,
                    jointType=pybullet.JOINT_POINT2POINT,
                    jointAxis=(0, 0, 0),
                    parentFramePosition=(0, 0, 0),
                    childFramePosition=(0, 0, 0),
                )
                # Store the constraint information
                self.constraints.update({constraint_name: constraint_id})

