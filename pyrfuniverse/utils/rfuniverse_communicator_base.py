import socket
import struct
from abc import ABC, abstractmethod
import numpy as np
from pyrfuniverse.utils.locker import Locker


class RFUniverseCommunicatorBase(ABC):
    def __init__(
            self,
            port: int = 5004,
            receive_data_callback=None,
            proc_type="editor",
    ):
        self.connected = False
        self.read_offset = 0
        self.on_receive_data = receive_data_callback
        self.port = port
        if proc_type == "editor":
            pass
        elif proc_type == "release":
            self._get_port()
        else:
            raise ValueError(f"Unknown proc_type: {proc_type}")

    def _get_port(self):
        with Locker("port"):
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            while self.port < 65536:
                try:
                    self.server.bind(("localhost", self.port))
                    self.server.close()
                    return
                except OSError:
                    self.port += 256
            raise OSError("No available port")

    def online(self):
        print(f"Connected successfully")
        self.connected = True
        self.receive_step()

    def close(self):
        self.connected = False

    def sync_step(self):
        self.send_object("StepStart")
        self.receive_step()

    def receive_step(self):
        while True:
            if not self.connected:
                raise ConnectionError("Connection closed")
            data = self.receive_bytes()
            if data is not None and len(data) > 0:
                objs = self.receive_objects(data)
                if len(objs) > 0 and objs[0] == "StepEnd":
                    break
                self.on_receive_data(objs)

    @abstractmethod
    def receive_bytes(self) -> bytes:
        pass

    @abstractmethod
    def send_bytes(self, data: bytes):
        pass

    def receive_objects(self, data: bytes) -> list:
        self.read_offset = 0
        count = self.read_int(data)
        objs = []
        for i in range(count):
            objs.append(self.read_object(data))
        return objs

    def read_object(self, datas: bytes) -> object:
        data_type = self.read_string(datas)
        if data_type == "int":
            return self.read_int(datas)
        elif data_type == "float":
            return self.read_float(datas)
        elif data_type == "string":
            return self.read_string(datas)
        elif data_type == "bool":
            return self.read_bool(datas)
        elif data_type == "bytes":
            return self.read_bytes(datas)
        elif data_type == "vector3":
            return self.read_object(datas)
        elif data_type == "quaternion":
            return self.read_object(datas)
        elif data_type == "matrix":
            return self.read_object(datas)
        elif data_type == "rect":
            return [self.read_float(datas) for _ in range(4)]
        elif data_type == "array":
            rank = self.read_int(datas)
            shape = []
            for _ in range(rank):
                shape.append(self.read_int(datas))
            result = np.ndarray(shape, dtype=np.float32)
            result = result.reshape(-1)
            for i in range(len(result)):
                result[i] = self.read_float(datas)
            return result.reshape(shape)
        elif data_type == "list":
            count = self.read_int(datas)
            result = []
            for _ in range(count):
                result.append(self.read_object(datas))
            return result
        elif data_type == "dict":
            count = self.read_int(datas)
            result = {}
            for _ in range(count):
                key = self.read_object(datas)
                value = self.read_object(datas)
                result[key] = value
            return result
        elif data_type == "tuple":
            count = self.read_int(datas)
            result = []
            for _ in range(count):
                result.append(self.read_object(datas))
            return tuple(result)
        elif data_type == "null" or data_type == "none":
            return None
        else:
            # print(f"dont support this type: {data_type}")
            raise ValueError(f"dont support this type: {data_type}")

    def read_string(self, datas: bytes) -> str:
        count = self.read_int(datas)
        try:
            assert count <= len(datas) - self.read_offset
        except:
            raise AssertionError(
                f"count: {count}, len(datas): {len(datas)}, self.read_offset: {self.read_offset}"
            )
        self.read_offset += count
        try:
            ret = datas[self.read_offset - count: self.read_offset].decode("utf-8")
        except:
            print(datas[self.read_offset - count: self.read_offset])
            print(
                f"read_start: {self.read_offset - count}, read_end: {self.read_offset}, count: {count}"
            )
            raise UnicodeDecodeError(
                "utf-8",
                datas[self.read_offset - count: self.read_offset],
                self.read_offset - count,
                self.read_offset,
            )
        return ret

    def read_int(self, datas: bytes) -> int:
        self.read_offset += 4
        ret = int.from_bytes(
            datas[self.read_offset - 4: self.read_offset],
            byteorder="little",
            signed=True,
        )
        return ret

    def read_float(self, datas: bytes) -> float:
        self.read_offset += 4
        return struct.unpack("f", datas[self.read_offset - 4: self.read_offset])[0]

    def read_bool(self, datas: bytes) -> bool:
        self.read_offset += 1
        return bool(
            int.from_bytes(
                datas[self.read_offset - 1: self.read_offset], byteorder="little"
            )
        )

    def read_bytes(self, datas: bytes) -> bytes:
        count = self.read_int(datas)
        self.read_offset += count
        return datas[self.read_offset - count: self.read_offset]

    def send_object(self, *args):
        datas = bytearray()
        self.write_int(datas, len(args))
        for obj in args:
            self.write_object(datas, obj)
        self.send_bytes(bytes(datas))

    def write_object(self, datas: bytearray, obj):
        if obj is None:
            self.write_string(datas, "none")
        elif type(obj) == int or type(obj) == np.int32 or type(obj) == np.int64:
            self.write_string(datas, "int")
            self.write_int(datas, obj)
        elif type(obj) == float or type(obj) == np.float32 or type(obj) == np.float64:
            self.write_string(datas, "float")
            self.write_float(datas, obj)
        elif type(obj) == bool:
            self.write_string(datas, "bool")
            self.write_bool(datas, obj)
        elif type(obj) == str:
            self.write_string(datas, "string")
            self.write_string(datas, obj)
        elif type(obj) == bytes or type(obj) == bytearray:
            self.write_string(datas, "bytes")
            self.write_bytes(datas, bytes(obj))
        elif type(obj) == list:
            self.write_string(datas, "list")
            self.write_int(datas, len(obj))
            for item in obj:
                self.write_object(datas, item)
        elif type(obj) == dict:
            self.write_string(datas, "dict")
            self.write_int(datas, len(obj))
            for item in obj:
                self.write_object(datas, item)
                self.write_object(datas, obj[item])
        elif type(obj) == np.ndarray:
            self.write_string(datas, "array")
            self.write_int(datas, len(obj.shape))
            for i in range(len(obj.shape)):
                self.write_int(datas, obj.shape[i])
            obj = obj.reshape(-1)
            for i in range(len(obj)):
                self.write_float(datas, float(obj[i]))
        elif type(obj) == tuple:
            self.write_string(datas, "tuple")
            self.write_int(datas, len(obj))
            for i in range(len(obj)):
                self.write_object(datas, obj[i])
        else:
            print(f"dont support this type: {type(obj)}")
            self.write_string(datas, "null")

    def write_string(self, datas: bytearray, s: str):
        s_byte = s.encode("utf-8")
        self.write_int(datas, len(s_byte))
        datas.extend(s_byte)

    def write_int(self, datas: bytearray, i: int):
        datas.extend(i.to_bytes(4, byteorder="little", signed=True))

    def write_float(self, datas: bytearray, f: float):
        datas.extend(struct.pack("f", f))

    def write_bool(self, datas: bytearray, b: bool):
        datas.extend(int(b).to_bytes(1, byteorder="little"))

    def write_bytes(self, datas: bytearray, b: bytes):
        self.write_int(datas, len(b))
        datas.extend(b)
