import pyrfuniverse.attributes as attr


class FallingClothAttr(attr.BaseAttr):
    def parse_message(self, data: dict):
        super().parse_message(data)

    def SetForceZoneParameters(
        self,
        orientation: float = None,
        intensity: float = None,
        turbulence: float = None,
        turbulence_frequency: float = None,
    ):
        self._send_data(
            "SetForceZoneParameters",
            orientation is not None,
            orientation,
            intensity is not None,
            intensity,
            turbulence is not None,
            turbulence,
            turbulence_frequency is not None,
            turbulence_frequency,
        )

    def SetSolverParameters(self, gravity: list):
        self._send_data("SetSolverParameters", gravity)
