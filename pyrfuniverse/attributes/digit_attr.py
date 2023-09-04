import pyrfuniverse.attributes as attr


class DigitAttr(attr.BaseAttr):
    """
    Class for simulating DIGIT tactile sensor.
    """

    def parse_message(self, data: dict):
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['light']: Bytes of RGB light image in DIGIT.

            self.data['depth']: Bytes of depth image in DIGIT.
        """
        super().parse_message(data)

    def GetData(self):
        """
        Get data from DIGIT in RFUniverse.

        """
        self._send_data("GetData")
