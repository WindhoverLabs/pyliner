# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: _py_EA_ChildData_t.proto

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
  name='_py_EA_ChildData_t.proto',
  package='',
  serialized_pb=_b('\n\x18_py_EA_ChildData_t.proto\"S\n\x11\x45\x41_ChildData_t_pb\x12\x16\n\x0e\x41ppInterpreter\x18\x01 \x02(\t\x12\x13\n\x0bucTlmHeader\x18\x02 \x03(\r\x12\x11\n\tAppScript\x18\x03 \x02(\t')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_EA_CHILDDATA_T_PB = _descriptor.Descriptor(
  name='EA_ChildData_t_pb',
  full_name='EA_ChildData_t_pb',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='AppInterpreter', full_name='EA_ChildData_t_pb.AppInterpreter', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ucTlmHeader', full_name='EA_ChildData_t_pb.ucTlmHeader', index=1,
      number=2, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='AppScript', full_name='EA_ChildData_t_pb.AppScript', index=2,
      number=3, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=_b("").decode('utf-8'),
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
  serialized_start=28,
  serialized_end=111,
)

DESCRIPTOR.message_types_by_name['EA_ChildData_t_pb'] = _EA_CHILDDATA_T_PB

EA_ChildData_t_pb = _reflection.GeneratedProtocolMessageType('EA_ChildData_t_pb', (_message.Message,), dict(
  DESCRIPTOR = _EA_CHILDDATA_T_PB,
  __module__ = '_py_EA_ChildData_t_pb2'
  # @@protoc_insertion_point(class_scope:EA_ChildData_t_pb)
  ))
_sym_db.RegisterMessage(EA_ChildData_t_pb)


# @@protoc_insertion_point(module_scope)
