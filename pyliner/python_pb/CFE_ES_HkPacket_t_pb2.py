# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: _py_CFE_ES_HkPacket_t.proto

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
  name='_py_CFE_ES_HkPacket_t.proto',
  package='',
  serialized_pb=_b('\n\x1b_py_CFE_ES_HkPacket_t.proto\"\x9f\x07\n\x1c\x43\x46\x45_ES_HkPacket_Payload_t_pb\x12\x17\n\x0fPerfTriggerMask\x18\x01 \x03(\r\x12\x16\n\x0ePerfFilterMask\x18\x02 \x03(\r\x12\x17\n\x0fProcessorResets\x18\x03 \x02(\r\x12\x12\n\nSysLogMode\x18\x04 \x02(\r\x12\x17\n\x0f\x43\x46\x45MinorVersion\x18\x05 \x02(\r\x12\x1a\n\x12MaxProcessorResets\x18\x06 \x02(\r\x12\x12\n\nErrCounter\x18\x07 \x02(\r\x12\x16\n\x0eRegisteredLibs\x18\x08 \x02(\r\x12\x13\n\x0b\x43\x46\x45Revision\x18\t \x02(\r\x12\x1e\n\x16RegisteredExternalApps\x18\n \x02(\r\x12\x1a\n\x12RegisteredCoreApps\x18\x0b \x02(\r\x12\x15\n\rHeapBytesFree\x18\x0c \x02(\r\x12\x12\n\nSysLogSize\x18\r \x02(\r\x12\x14\n\x0cResetSubtype\x18\x0e \x02(\r\x12\x1b\n\x13OSALMissionRevision\x18\x0f \x02(\r\x12\x13\n\x0bPerfDataEnd\x18\x10 \x02(\r\x12\x15\n\rPerfDataStart\x18\x11 \x02(\r\x12\x12\n\nBootSource\x18\x12 \x02(\r\x12\x18\n\x10PerfTriggerCount\x18\x13 \x02(\r\x12\x11\n\tPerfState\x18\x14 \x02(\r\x12\x18\n\x10HeapMaxBlockSize\x18\x15 \x02(\r\x12\x14\n\x0c\x45RLogEntries\x18\x16 \x02(\r\x12\x17\n\x0fSysLogBytesUsed\x18\x17 \x02(\r\x12\x1a\n\x12\x43\x46\x45MissionRevision\x18\x18 \x02(\r\x12\x17\n\x0fRegisteredTasks\x18\x19 \x02(\r\x12\x18\n\x10OSALMinorVersion\x18\x1a \x02(\r\x12\x12\n\nCmdCounter\x18\x1b \x02(\r\x12\x18\n\x10OSALMajorVersion\x18\x1c \x02(\r\x12\x17\n\x0f\x43\x46\x45\x43oreChecksum\x18\x1d \x02(\r\x12\x12\n\nERLogIndex\x18\x1e \x02(\r\x12\x17\n\x0fPerfDataToWrite\x18\x1f \x02(\r\x12\x17\n\x0f\x43\x46\x45MajorVersion\x18  \x02(\r\x12\x15\n\rSysLogEntries\x18! \x02(\r\x12\x14\n\x0cOSALRevision\x18\" \x02(\r\x12\x16\n\x0eHeapBlocksFree\x18# \x02(\r\x12\x10\n\x08PerfMode\x18$ \x02(\r\x12\x11\n\tResetType\x18% \x02(\r\x12\x15\n\rPerfDataCount\x18& \x02(\r\"Y\n\x14\x43\x46\x45_ES_HkPacket_t_pb\x12\x11\n\tTlmHeader\x18\x01 \x03(\r\x12.\n\x07Payload\x18\x02 \x02(\x0b\x32\x1d.CFE_ES_HkPacket_Payload_t_pb')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)




_CFE_ES_HKPACKET_PAYLOAD_T_PB = _descriptor.Descriptor(
  name='CFE_ES_HkPacket_Payload_t_pb',
  full_name='CFE_ES_HkPacket_Payload_t_pb',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='PerfTriggerMask', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfTriggerMask', index=0,
      number=1, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='PerfFilterMask', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfFilterMask', index=1,
      number=2, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ProcessorResets', full_name='CFE_ES_HkPacket_Payload_t_pb.ProcessorResets', index=2,
      number=3, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='SysLogMode', full_name='CFE_ES_HkPacket_Payload_t_pb.SysLogMode', index=3,
      number=4, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='CFEMinorVersion', full_name='CFE_ES_HkPacket_Payload_t_pb.CFEMinorVersion', index=4,
      number=5, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='MaxProcessorResets', full_name='CFE_ES_HkPacket_Payload_t_pb.MaxProcessorResets', index=5,
      number=6, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ErrCounter', full_name='CFE_ES_HkPacket_Payload_t_pb.ErrCounter', index=6,
      number=7, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='RegisteredLibs', full_name='CFE_ES_HkPacket_Payload_t_pb.RegisteredLibs', index=7,
      number=8, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='CFERevision', full_name='CFE_ES_HkPacket_Payload_t_pb.CFERevision', index=8,
      number=9, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='RegisteredExternalApps', full_name='CFE_ES_HkPacket_Payload_t_pb.RegisteredExternalApps', index=9,
      number=10, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='RegisteredCoreApps', full_name='CFE_ES_HkPacket_Payload_t_pb.RegisteredCoreApps', index=10,
      number=11, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='HeapBytesFree', full_name='CFE_ES_HkPacket_Payload_t_pb.HeapBytesFree', index=11,
      number=12, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='SysLogSize', full_name='CFE_ES_HkPacket_Payload_t_pb.SysLogSize', index=12,
      number=13, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ResetSubtype', full_name='CFE_ES_HkPacket_Payload_t_pb.ResetSubtype', index=13,
      number=14, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='OSALMissionRevision', full_name='CFE_ES_HkPacket_Payload_t_pb.OSALMissionRevision', index=14,
      number=15, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='PerfDataEnd', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfDataEnd', index=15,
      number=16, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='PerfDataStart', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfDataStart', index=16,
      number=17, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='BootSource', full_name='CFE_ES_HkPacket_Payload_t_pb.BootSource', index=17,
      number=18, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='PerfTriggerCount', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfTriggerCount', index=18,
      number=19, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='PerfState', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfState', index=19,
      number=20, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='HeapMaxBlockSize', full_name='CFE_ES_HkPacket_Payload_t_pb.HeapMaxBlockSize', index=20,
      number=21, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ERLogEntries', full_name='CFE_ES_HkPacket_Payload_t_pb.ERLogEntries', index=21,
      number=22, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='SysLogBytesUsed', full_name='CFE_ES_HkPacket_Payload_t_pb.SysLogBytesUsed', index=22,
      number=23, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='CFEMissionRevision', full_name='CFE_ES_HkPacket_Payload_t_pb.CFEMissionRevision', index=23,
      number=24, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='RegisteredTasks', full_name='CFE_ES_HkPacket_Payload_t_pb.RegisteredTasks', index=24,
      number=25, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='OSALMinorVersion', full_name='CFE_ES_HkPacket_Payload_t_pb.OSALMinorVersion', index=25,
      number=26, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='CmdCounter', full_name='CFE_ES_HkPacket_Payload_t_pb.CmdCounter', index=26,
      number=27, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='OSALMajorVersion', full_name='CFE_ES_HkPacket_Payload_t_pb.OSALMajorVersion', index=27,
      number=28, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='CFECoreChecksum', full_name='CFE_ES_HkPacket_Payload_t_pb.CFECoreChecksum', index=28,
      number=29, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ERLogIndex', full_name='CFE_ES_HkPacket_Payload_t_pb.ERLogIndex', index=29,
      number=30, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='PerfDataToWrite', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfDataToWrite', index=30,
      number=31, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='CFEMajorVersion', full_name='CFE_ES_HkPacket_Payload_t_pb.CFEMajorVersion', index=31,
      number=32, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='SysLogEntries', full_name='CFE_ES_HkPacket_Payload_t_pb.SysLogEntries', index=32,
      number=33, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='OSALRevision', full_name='CFE_ES_HkPacket_Payload_t_pb.OSALRevision', index=33,
      number=34, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='HeapBlocksFree', full_name='CFE_ES_HkPacket_Payload_t_pb.HeapBlocksFree', index=34,
      number=35, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='PerfMode', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfMode', index=35,
      number=36, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ResetType', full_name='CFE_ES_HkPacket_Payload_t_pb.ResetType', index=36,
      number=37, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='PerfDataCount', full_name='CFE_ES_HkPacket_Payload_t_pb.PerfDataCount', index=37,
      number=38, type=13, cpp_type=3, label=2,
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
  serialized_end=959,
)


_CFE_ES_HKPACKET_T_PB = _descriptor.Descriptor(
  name='CFE_ES_HkPacket_t_pb',
  full_name='CFE_ES_HkPacket_t_pb',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='TlmHeader', full_name='CFE_ES_HkPacket_t_pb.TlmHeader', index=0,
      number=1, type=13, cpp_type=3, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='Payload', full_name='CFE_ES_HkPacket_t_pb.Payload', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
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
  serialized_start=961,
  serialized_end=1050,
)

_CFE_ES_HKPACKET_T_PB.fields_by_name['Payload'].message_type = _CFE_ES_HKPACKET_PAYLOAD_T_PB
DESCRIPTOR.message_types_by_name['CFE_ES_HkPacket_Payload_t_pb'] = _CFE_ES_HKPACKET_PAYLOAD_T_PB
DESCRIPTOR.message_types_by_name['CFE_ES_HkPacket_t_pb'] = _CFE_ES_HKPACKET_T_PB

CFE_ES_HkPacket_Payload_t_pb = _reflection.GeneratedProtocolMessageType('CFE_ES_HkPacket_Payload_t_pb', (_message.Message,), dict(
  DESCRIPTOR = _CFE_ES_HKPACKET_PAYLOAD_T_PB,
  __module__ = '_py_CFE_ES_HkPacket_t_pb2'
  # @@protoc_insertion_point(class_scope:CFE_ES_HkPacket_Payload_t_pb)
  ))
_sym_db.RegisterMessage(CFE_ES_HkPacket_Payload_t_pb)

CFE_ES_HkPacket_t_pb = _reflection.GeneratedProtocolMessageType('CFE_ES_HkPacket_t_pb', (_message.Message,), dict(
  DESCRIPTOR = _CFE_ES_HKPACKET_T_PB,
  __module__ = '_py_CFE_ES_HkPacket_t_pb2'
  # @@protoc_insertion_point(class_scope:CFE_ES_HkPacket_t_pb)
  ))
_sym_db.RegisterMessage(CFE_ES_HkPacket_t_pb)


# @@protoc_insertion_point(module_scope)
