# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: _py_SC_RtsInfoEntry_t.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='_py_SC_RtsInfoEntry_t.proto',
  package='',
  serialized_pb=_b('\n\x1b_py_SC_RtsInfoEntry_t.proto\"\xa3\x01\n\x14SC_RtsInfoEntry_t_pb\x12\x17\n\x0fNextCommandTime\x18\x01 \x02(\r\x12\x11\n\tCmdErrCtr\x18\x02 \x02(\r\x12\x0e\n\x06UseCtr\x18\x03 \x02(\r\x12\x14\n\x0c\x44isabledFlag\x18\x04 \x02(\x08\x12\x16\n\x0eNextCommandPtr\x18\x05 \x02(\r\x12\x11\n\tRtsStatus\x18\x06 \x02(\r\x12\x0e\n\x06\x43mdCtr\x18\x07 \x02(\r')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_SC_RTSINFOENTRY_T_PB = _descriptor.Descriptor(
  name='SC_RtsInfoEntry_t_pb',
  full_name='SC_RtsInfoEntry_t_pb',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='NextCommandTime', full_name='SC_RtsInfoEntry_t_pb.NextCommandTime', index=0,
      number=1, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='CmdErrCtr', full_name='SC_RtsInfoEntry_t_pb.CmdErrCtr', index=1,
      number=2, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='UseCtr', full_name='SC_RtsInfoEntry_t_pb.UseCtr', index=2,
      number=3, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='DisabledFlag', full_name='SC_RtsInfoEntry_t_pb.DisabledFlag', index=3,
      number=4, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='NextCommandPtr', full_name='SC_RtsInfoEntry_t_pb.NextCommandPtr', index=4,
      number=5, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='RtsStatus', full_name='SC_RtsInfoEntry_t_pb.RtsStatus', index=5,
      number=6, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='CmdCtr', full_name='SC_RtsInfoEntry_t_pb.CmdCtr', index=6,
      number=7, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=32,
  serialized_end=195,
)

DESCRIPTOR.message_types_by_name['SC_RtsInfoEntry_t_pb'] = _SC_RTSINFOENTRY_T_PB

SC_RtsInfoEntry_t_pb = _reflection.GeneratedProtocolMessageType('SC_RtsInfoEntry_t_pb', (_message.Message,), dict(
  DESCRIPTOR = _SC_RTSINFOENTRY_T_PB,
  __module__ = '_py_SC_RtsInfoEntry_t_pb2'
  # @@protoc_insertion_point(class_scope:SC_RtsInfoEntry_t_pb)
  ))
_sym_db.RegisterMessage(SC_RtsInfoEntry_t_pb)


# @@protoc_insertion_point(module_scope)
