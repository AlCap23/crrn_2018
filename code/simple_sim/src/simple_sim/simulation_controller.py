import pybullet


class _BaseControl(object):
    def __init__(self, robot, joint, addSlider=False):
        self.robot = robot
        self.joint = joint
        self.joint_id = self.robot.joints[joint]["jointIndex"]
        self.client_id = self.robot.client_id
        self.max_force = self.robot.joints[joint]["jointMaxForce"]
        self.max_velocity = self.robot.joints[joint]["jointMaxVelocity"]
        self.reference = None
        self.slider = addSlider

        robot.addController(self)

    def _addSlider(self):
        if not hasattr(self, "_slider"):
            self._slider = -1

        if self.type == pybullet.POSITION_CONTROL:
            param_name = self.joint + "_Position"
            param_val = pybullet.getJointState(
                self.robot.id, self.joint_id, self.client_id
            )[0]
            minVal = self.robot.joints[self.joint]["jointLowerLimit"]
            maxVal = self.robot.joints[self.joint]["jointUpperLimit"]
        elif self.type == pybullet.VELOCITY_CONTROL:
            param_name = self.joint + "_Velocity"
            param_val = pybullet.getJointState(
                self.robot.id, self.joint_id, self.client_id
            )[1]
            minVal = -1. * abs(self.max_velocity)
            maxVal = abs(self.max_velocity)
        elif self.type == pybullet.TORQUE_CONTROL:
            param_name = self.joint + "_Torque"
            param_val = pybullet.getJointState(
                self.robot.id, self.joint_id, self.client_id
            )[3]
            minVal = -1. * abs(self.max_force)
            maxVal = abs(self.max_force)

        self._slider = pybullet.addUserDebugParameter(
            paramName=param_name,
            rangeMin=minVal,
            rangeMax=maxVal,
            startValue=param_val,
            physicsClientId=self.client_id,
        )

        print(self._slider)

    def setReference(self, reference):

        self.reference = reference

    def update(self):

        controlArgs = {
            "bodyUniqueId": self.robot.id,
            "jointIndex": self.joint_id,
            "controlMode": self.type,
            "maxVelocity": self.max_velocity,
            "force": self.max_force,
            "physicsClientId": self.client_id,
        }

        if self.slider:
            self.reference = pybullet.readUserDebugParameter(
                self._slider, self.client_id
            )

        if self.reference:
            if self.type == pybullet.POSITION_CONTROL:
                controlArgs.update({"targetPosition": self.reference})
            elif self.type == pybullet.VELOCITY_CONTROL:
                controlArgs.update({"targetVelocity": self.reference})
            elif self.type == pybullet.TORQUE_CONTROL:
                controlArgs.update({"force": self.reference})

        pybullet.setJointMotorControl2(**controlArgs)


class PositionControl(_BaseControl):
    """ Simple position controller.
    """

    def __init__(self, robot, joint, addSlider=False):
        super().__init__(robot, joint, addSlider)

        self.type = pybullet.POSITION_CONTROL

        if self.slider:
            self._addSlider()


class VelocityControl(_BaseControl):
    """ Simple position controller.
    """

    def __init__(self, robot, joint, addSlider=False):
        super().__init__(robot, joint, addSlider)

        self.type = pybullet.VELOCITY_CONTROL

        if self.slider:
            self._addSlider()


class TorqueControl(_BaseControl):
    """ Simple position controller.
    """

    def __init__(self, robot, joint, addSlider=False):
        super().__init__(robot, joint, addSlider)

        self.type = pybullet.TORQUE_CONTROL

        if self.slider:
            self._addSlider()
