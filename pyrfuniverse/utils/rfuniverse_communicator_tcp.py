import platform
import socket
from pyrfuniverse.utils.rfuniverse_communicator_base import RFUniverseCommunicatorBase


class RFUniverseCommunicatorTCP(RFUniverseCommunicatorBase):
    def __init__(
            self,
            port: int = 5004,
            receive_data_callback=None,
            get_port=False,
    ):
        super().__init__(
            port=port,
            receive_data_callback=receive_data_callback,
            get_port=get_port)
        self.server = None
        self.client = None

    def online(self):
        print(f"Waiting for connections on port: {self.port}...")
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(("0.0.0.0", self.port))
        self.server.listen(1)
        self.client, _ = self.server.accept()
        self.client.settimeout(None)
        self.client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        super().online()

    def close(self):
        if self.client is not None:
            self.client.close()
        if self.server is not None:
            self.server.close()

        super().close()

    def receive_bytes(self):
        data = bytearray()
        while len(data) < 4:
            temp_data = self.client.recv(4 - len(data))
            assert len(temp_data) != 0
            data.extend(temp_data)
        assert len(data) == 4
        length = int.from_bytes(data, byteorder="little", signed=False)
        if length == 0:
            return None
        buffer = bytearray()
        while len(buffer) < length:
            temp_data = self.client.recv(length - len(buffer))
            assert len(temp_data) != 0
            buffer.extend(temp_data)
        assert len(buffer) == length
        return buffer

    def send_bytes(self, data: bytes):
        if not self.connected:
            return
        length = len(data).to_bytes(4, byteorder="little", signed=False)
        send_len = 0
        while send_len < 4:
            send_len += self.client.send(length)
        send_len = 0
        while send_len < len(data):
            send_len += self.client.send(data)
        if platform == 'linux':
            self.client.setsockopt(socket.IPPROTO_TCP, socket.TCP_QUICKACK, 1)
