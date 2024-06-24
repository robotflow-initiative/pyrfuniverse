import queue
import grpc
import pyrfuniverse.grpc.RFUniverseGRPC_pb2 as RFUniverseGRPC_pb2
import pyrfuniverse.grpc.RFUniverseGRPC_pb2_grpc as RFUniverseGRPC_pb2_grpc


class RFUniverseGrpcServer(RFUniverseGRPC_pb2_grpc.GrpcServiceServicer):
    def __init__(self):
        super().__init__()
        self.connected = False
        self.send_queue = queue.Queue()
        self.receive_queue = queue.Queue()

    def Link(self, request, context):
        return RFUniverseGRPC_pb2.Empty()

    def CSharpToPythonStream(self, request_iterator, context):
        for request in request_iterator:
            if not self.connected:
                return
            self.receive_queue.put(request.data)

    def PythonToCSharpStream(self, request, context):
        self.connected = True
        while self.connected:
            try:
                response = self.send_queue.get(timeout=1.)
            except queue.Empty:
                continue
            try:
                yield RFUniverseGRPC_pb2.BinaryMessage(data=response)
            except grpc.RpcError:
                self.connected = False
                break
