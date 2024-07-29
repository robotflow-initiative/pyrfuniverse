import time
from concurrent import futures
import grpc
from pyrfuniverse.utils.rfuniverse_communicator_base import RFUniverseCommunicatorBase
import pyrfuniverse.grpc.RFUniverseGRPC_pb2_grpc as RFUniverseGRPC_pb2_grpc
from pyrfuniverse.grpc.RFUniverseGRPC_Server import RFUniverseGrpcServer


class RFUniverseCommunicatorGRPC(RFUniverseCommunicatorBase):
    def __init__(
            self,
            port: int = 5004,
            receive_data_callback=None,
            get_port=False,
    ):
        super().__init__(
            port=port,
            receive_data_callback=receive_data_callback,
            get_port=get_port,
        )
        self.grpc_server = None
        self.rfuniverse_grpc_server = None

    def online(self):
        print(f"Waiting for connections on port: {self.port}...")
        self.grpc_server = grpc.server(futures.ThreadPoolExecutor(max_workers=5))
        self.rfuniverse_grpc_server = RFUniverseGrpcServer()
        RFUniverseGRPC_pb2_grpc.add_GrpcServiceServicer_to_server(self.rfuniverse_grpc_server,
                                                                  self.grpc_server)
        self.grpc_server.add_insecure_port(f'0.0.0.0:{self.port}')
        self.grpc_server.start()
        while not self.rfuniverse_grpc_server.connected:
            time.sleep(0.1)

        super().online()

    def close(self):
        if self.grpc_server is not None:
            self.grpc_server.stop(0)
            self.rfuniverse_grpc_server.connected = False

        super().close()

    def receive_bytes(self):
        return self.rfuniverse_grpc_server.receive_queue.get()

    def send_bytes(self, data: bytes):
        self.rfuniverse_grpc_server.send_queue.put(data)
