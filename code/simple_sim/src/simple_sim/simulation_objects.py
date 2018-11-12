import pybullet
import pybullet_data


class SimulationObject(object):
    """Base class for simulation objects
    """

    def __init__(self, server, urdf=None, robot=False, **kwargs):
        # Client of the simulation
        self.client_id = server.client_id
        self.urdf_file = urdf

        self._loadUrdf(**kwargs)
        self._initGeneralParameter()

        if not robot:
            server.addObject(self)
        else:
            server.addRobot(self)

    def _loadUrdf(self, *args, **kwargs):
        """Load the associated urdf file.
        """

        allowedKeys = [
            "basePosition",
            "baseOrientation",
            "useMaximalCoordinates",
            "useFixedBase",
            "flags",
            "globalScaling",
        ]

        loadKeys = {"fileName": self.urdf_file, "physicsClientId": self.client_id}

        for additionalKey in kwargs.keys():
            if additionalKey in allowedKeys:
                loadKeys.update({additionalKey: kwargs[additionalKey]})

        self.id = pybullet.loadURDF(**loadKeys)

    def _initGeneralParameter(self):
        """ Initializes the general parameter, e.g. joint_dict, link_dict.
        """

        self.link_no = pybullet.getNumJoints(
            bodyUniqueId=self.id, physicsClientId=self.client_id
        )

        for i in range(self.link_no):
            self._storeJointParameter(i)

        # Get the base
        self._storeLinkParameter(-1)
        if hasattr(self, "_links"):
            for link in self._links.keys():
                self._storeLinkParameter(link)

    def _storeLinkParameter(self, link):
        """Stores and orderes the parameter of a link as an internal dict.
        """

        if not hasattr(self, "links"):
            self.links = {}
        if not hasattr(self, "_links"):
            self._links = {}

        # Take only valid joints
        if link == -1:
            link_id = -1
            link = "base"
            self._links.update({link: link_id})
        else:
            if not self._links[link]:
                return
            link_id = self._links[link]

        param_keys = [
            "mass",
            "lateral_friction",
            "local_inertial_diagonal",
            "local_inertial_pos",
            "local_inertial_orn",
            "restitution",
            "rolling_friction",
            "spinning_friction",
            "contact_damping",
            "contact_stiffness",
        ]

        param_dict = {}
        for param_key, param_val in zip(
            param_keys, pybullet.getDynamicsInfo(self.id, link_id, self.client_id)
        ):
            param_dict.update({param_key: param_val})

        self.links.update({link: param_dict})

    def _storeJointParameter(self, joint_id):

        if not hasattr(self, "joints"):
            self.joints = {}

        if not hasattr(self, "_links"):
            self._links = {}

        # Take only valid joints
        if not (joint_id >= 0 and joint_id <= self.link_no):
            return

        param_keys = [
            "jointIndex",
            "jointName",
            "jointType",
            "qIndex",
            "uIndex",
            "flags",
            "jointDamping",
            "jointFricition",
            "jointLowerLimit",
            "jointUpperLimit",
            "jointMaxForce",
            "jointMaxVelocity",
            "linkName",
            "parentFramePos",
            "parentFrameOrn",
            "parentIndex",
        ]

        param_dict = {}
        for param_key, param_val in zip(
            param_keys, pybullet.getJointInfo(self.id, joint_id, self.client_id)
        ):
            param_dict.update({param_key: param_val})

        self.joints.update({param_dict["jointName"].decode("utf-8"): param_dict})
        self._links.update({param_dict["linkName"].decode("utf-8"): joint_id})

    def changeDynamicParameter(self, link, **kwargs):
        """Change the parameter of a link.
        """
        # Take only valid links
        if link in self._links.keys():
            link_id = self._links[link]
        else:
            return

        allowedKeys = [
            "mass",
            "restitution",
            "contactStiffness",
            "contactDamping",
            "localInertiaDiagonal",
        ]

        loadKeys = {
            "bodyUniqueId": self.id,
            "linkIndex": link_id,
            "physicsClientId": self.client_id,
        }

        for additionalKey in kwargs.keys():
            if additionalKey in allowedKeys:
                loadKeys.update({additionalKey: kwargs[additionalKey]})

        pybullet.changeDynamics(**loadKeys)
        self._storeLinkParameter(link_id)

    def getMeasurements(self):
        # Get all measured variables
        self._collectJointMeasurements()
        self._collectLinkMeasurements()

        return self._measurements

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

                loadKeys = {
                    "bodyUniqueId": self.id,
                    "linkIndex": self._links[link],
                    "physicsClientId": self.client_id,
                }

                if "linear_velocity" in variables or "angular_velocity" in variables:
                    loadKeys.update({"computeLinkVelocity": 1})
                    pos_glo, orn_glo, com_pos_loc, com_orn_loc, frame_pos_glo, frame_orn_glo, vel_lin_glo, vel_ang_glo = pybullet.getLinkState(
                        **loadKeys
                    )
                else:
                    pos_glo, orn_glo, com_pos_loc, com_orn_loc, frame_pos_glo, frame_orn_glo = pybullet.getLinkState(
                        **loadKeys
                    )

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

