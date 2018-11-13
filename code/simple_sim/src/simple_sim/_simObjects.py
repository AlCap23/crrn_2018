import pybullet
import pybullet_data


class _SimObject(object):
    """Base class for simulation objects which do not need to be controlled.

    Args:
     server, object : Simulation server
     file, path : Path to the urdf file
     kwargs : Additional parameter as specified by pybullet.loadURDF
    
    Returns:
     None
    """

    def __init__(self, server, urdf=None, **kwargs):
        # Client of the simulation
        self.client_id = server.client_id
        self.urdf_file = urdf

        self._loadUrdf(**kwargs)
        self._initGeneralParameter()

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

    def changeDynamicParameter(self, link, add_slider=False, **kwargs):
        """Change the parameter of a link.
        """
        # Take only valid links
        if link in self._links.keys():
            link_id = self._links[link]
        else:
            return

        allowedKeys = {
            "mass": [0, 1e1],
            "restitution": [0, 0.99],
            "contactStiffness": [0, 0.3],
            "contactDamping": [0, 0.3],
            "localInertiaDiagonal": [],
        }

        loadKeys = {
            "bodyUniqueId": self.id,
            "linkIndex": link_id,
            "physicsClientId": self.client_id,
        }

        for additionalKey in kwargs.keys():
            if additionalKey in allowedKeys:
                loadKeys.update({additionalKey: kwargs[additionalKey]})

                if add_slider and additionalKey is not "localInertiaDiagonal":
                    if not hasattr(self, "_sliders"):
                        self._sliders = {}

                    param_name = link + "_" + additionalKey
                    param_val = kwargs[additionalKey]

                    self._sliders.update(
                        {
                            param_name: pybullet.addUserDebugParameter(
                                paramName=param_name,
                                rangeMin=allowedKeys[additionalKey][0],
                                rangeMax=allowedKeys[additionalKey][1],
                                startValue=param_val,
                                physicsClientId=self.client_id,
                            )
                        }
                    )

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

