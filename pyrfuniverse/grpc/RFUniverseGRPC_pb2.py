# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: RFUniverseGRPC.proto
# Protobuf Python Version: 4.25.1
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x14RFUniverseGRPC.proto\x12\nRFUniverse\"\x07\n\x05\x45mpty\"\x1d\n\rBinaryMessage\x12\x0c\n\x04\x64\x61ta\x18\x01 \x01(\x0c\x32\xcb\x01\n\x0bGrpcService\x12,\n\x04Link\x12\x11.RFUniverse.Empty\x1a\x11.RFUniverse.Empty\x12\x46\n\x14\x43SharpToPythonStream\x12\x19.RFUniverse.BinaryMessage\x1a\x11.RFUniverse.Empty(\x01\x12\x46\n\x14PythonToCSharpStream\x12\x11.RFUniverse.Empty\x1a\x19.RFUniverse.BinaryMessage0\x01\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'RFUniverseGRPC_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_EMPTY']._serialized_start=36
  _globals['_EMPTY']._serialized_end=43
  _globals['_BINARYMESSAGE']._serialized_start=45
  _globals['_BINARYMESSAGE']._serialized_end=74
  _globals['_GRPCSERVICE']._serialized_start=77
  _globals['_GRPCSERVICE']._serialized_end=280
# @@protoc_insertion_point(module_scope)