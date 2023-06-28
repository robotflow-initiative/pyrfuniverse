import pyrfuniverse.attributes as attr


class ColliderAttr(attr.GameObjectAttr):
    """
    Collider class for objects who have collider in Unity.
    """
    def parse_message(self, data: dict):
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.
        """
        super().parse_message(data)

    def GenerateVHACDColider(self):
        """
        Generate convex colliders using VHACD algorithm.
        """
        self._send_data('GenerateVHACDColider')