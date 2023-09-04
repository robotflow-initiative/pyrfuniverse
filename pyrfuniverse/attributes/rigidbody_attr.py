import pyrfuniverse.attributes as attr


class RigidbodyAttr(attr.ColliderAttr):
    """
    Rigid body class.
    """

    def parse_message(self, data: dict):
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['velocity']: The velocity of the object.

            self.data['angular_vel']: The angular velcity of the object.
        """
        super().parse_message(data)

    def SetMass(self, mass: float):
        """
        Set the mass of this rigid body object

        Args:
            mass: Float, representing the mass of this rigid body.
        """
        self._send_data("SetMass", float(mass))

    def AddForce(self, force: list):
        """
        Add force to this rigid body object.

        Args:
            force: A list of length 3, representing the force added to this rigid body.
        """
        if force is not None:
            force = [float(i) for i in force]

        self._send_data("AddForce", force)

    def SetVelocity(self, velocity: list):
        """
        Set the velocity of this rigid body object.

        Args:
            velocity: A list of length 3, representing the velocity of this rigid body.
        """
        if velocity is not None:
            velocity = [float(i) for i in velocity]

        self._send_data("SetVelocity", velocity)

    def SetKinematic(self, is_kinematic: bool):
        """
        Set the Rigidbody is kinematic or not.

        Args:
            is_kinematic: is kinematic or not.
        """
        self._send_data("SetKinematic", is_kinematic)
