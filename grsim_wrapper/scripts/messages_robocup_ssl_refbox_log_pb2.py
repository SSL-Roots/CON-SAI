# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: messages_robocup_ssl_refbox_log.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import messages_robocup_ssl_detection_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='messages_robocup_ssl_refbox_log.proto',
  package='',
  serialized_pb='\n%messages_robocup_ssl_refbox_log.proto\x1a$messages_robocup_ssl_detection.proto\"C\n\tLog_Frame\x12\"\n\x05\x66rame\x18\x01 \x02(\x0b\x32\x13.SSL_DetectionFrame\x12\x12\n\nrefbox_cmd\x18\x02 \x02(\t\"%\n\nRefbox_Log\x12\x17\n\x03log\x18\x01 \x03(\x0b\x32\n.Log_Frame')




_LOG_FRAME = _descriptor.Descriptor(
  name='Log_Frame',
  full_name='Log_Frame',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frame', full_name='Log_Frame.frame', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='refbox_cmd', full_name='Log_Frame.refbox_cmd', index=1,
      number=2, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=unicode("", "utf-8"),
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
  serialized_start=79,
  serialized_end=146,
)


_REFBOX_LOG = _descriptor.Descriptor(
  name='Refbox_Log',
  full_name='Refbox_Log',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='log', full_name='Refbox_Log.log', index=0,
      number=1, type=11, cpp_type=10, label=3,
      has_default_value=False, default_value=[],
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
  serialized_start=148,
  serialized_end=185,
)

_LOG_FRAME.fields_by_name['frame'].message_type = messages_robocup_ssl_detection_pb2._SSL_DETECTIONFRAME
_REFBOX_LOG.fields_by_name['log'].message_type = _LOG_FRAME
DESCRIPTOR.message_types_by_name['Log_Frame'] = _LOG_FRAME
DESCRIPTOR.message_types_by_name['Refbox_Log'] = _REFBOX_LOG

class Log_Frame(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _LOG_FRAME

  # @@protoc_insertion_point(class_scope:Log_Frame)

class Refbox_Log(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _REFBOX_LOG

  # @@protoc_insertion_point(class_scope:Refbox_Log)


# @@protoc_insertion_point(module_scope)
