import pyrfuniverse.attributes as attr


class ClothAttr(attr.BaseAttr):
    """
    Obi cloth class.
    """

    def parse_message(self, data: dict):
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

        """
        super().parse_message(data)
