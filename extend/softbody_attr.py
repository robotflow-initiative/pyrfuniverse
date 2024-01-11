import pyrfuniverse.attributes as attr


class SoftbodyAttr(attr.BaseAttr):
    """
    Obi Softbody class
    """
    def parse_message(self, data: dict):
        """
        Parse messages. This function is called by internal function.
        """
        super().parse_message(data)
