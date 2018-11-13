from simple_sim._simObjects import _SimObject

import pybullet


class SimObject(_SimObject):
    """Base class for a simulation object.
    """

    def __init__(self, server, urdf=None, **kwargs):
        super().__init__(server, urdf, **kwargs)

        # Add the robot to the server
        server.addObject(self)


class SimHeightmap(object):
    def __init__(self, server, mesh, **kwargs):
        # Client of the simulation
        self.client_id = server.client_id
        self.mesh_file = mesh

        self._createHeightmap(**kwargs)
        self._initGeneralParameter()

        server.addObject(self)

    def update(self):
        self._updateSliderValues()

    def _createHeightmap(self, **kwargs):

        loadKeys = {
            "shapeType": pybullet.GEOM_MESH,
            "fileName": self.mesh_file,
            "flags": pybullet.GEOM_FORCE_CONCAVE_TRIMESH
            | pybullet.GEOM_CONCAVE_INTERNAL_EDGE,
        }

        if "scale" in kwargs:
            loadKeys.update({"meshScale": kwargs["scale"]})

        # Create the terrain via the obj data
        terrain_collision_shape = pybullet.createCollisionShape(**loadKeys)

        # Create the heightmap
        loadKeys = {
            "baseMass": 0,
            "baseCollisionShapeIndex": terrain_collision_shape,
            "baseVisualShapeIndex": -1,
            "basePosition": [0, 0, 0],
            "baseOrientation": [0, 0, 0, 1],
        }

        if "basePosition" in kwargs:
            loadKeys.update({"basePosition": kwargs["basePosition"]})
        if "baseOrientation" in kwargs:
            loadKeys.update(
                {
                    "baseOrientation": pybullet.getQuaternionFromEuler(
                        kwargs["baseOrientation"]
                    )
                }
            )

        self.id = pybullet.createMultiBody(**loadKeys)

        if "texture" in kwargs:
            self.texture_id = pybullet.loadTexture(kwargs["texture"])
            print(self.texture_id)
            pybullet.changeVisualShape(self.id, -1, -1, self.texture_id)

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

    def _updateSliderValues(self):
        if not hasattr(self, "_sliders"):
            return

        for slider, slider_id in self._sliders.items():
            # Split the slider name to get the id
            name, param = slider.split("_")
            # Get the value
            value = pybullet.readUserDebugParameter(slider_id, self.client_id)
            self.changeDynamicParameter(
                **{"link": name, "add_slider": False, param: value}
            )
