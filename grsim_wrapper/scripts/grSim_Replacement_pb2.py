# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: grSim_Replacement.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)




DESCRIPTOR = _descriptor.FileDescriptor(
  name='grSim_Replacement.proto',
  package='',
  serialized_pb='\n\x17grSim_Replacement.proto\"[\n\x16grSim_RobotReplacement\x12\t\n\x01x\x18\x01 \x02(\x01\x12\t\n\x01y\x18\x02 \x02(\x01\x12\x0b\n\x03\x64ir\x18\x03 \x02(\x01\x12\n\n\x02id\x18\x04 \x02(\r\x12\x12\n\nyellowteam\x18\x05 \x02(\x08\"E\n\x15grSim_BallReplacement\x12\t\n\x01x\x18\x01 \x02(\x01\x12\t\n\x01y\x18\x02 \x02(\x01\x12\n\n\x02vx\x18\x03 \x02(\x01\x12\n\n\x02vy\x18\x04 \x02(\x01\"b\n\x11grSim_Replacement\x12$\n\x04\x62\x61ll\x18\x01 \x01(\x0b\x32\x16.grSim_BallReplacement\x12\'\n\x06robots\x18\x02 \x03(\x0b\x32\x17.grSim_RobotReplacement')




_GRSIM_ROBOTREPLACEMENT = _descriptor.Descriptor(
  name='grSim_RobotReplacement',
  full_name='grSim_RobotReplacement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='grSim_RobotReplacement.x', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='grSim_RobotReplacement.y', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='dir', full_name='grSim_RobotReplacement.dir', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='id', full_name='grSim_RobotReplacement.id', index=3,
      number=4, type=13, cpp_type=3, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='yellowteam', full_name='grSim_RobotReplacement.yellowteam', index=4,
      number=5, type=8, cpp_type=7, label=2,
      has_default_value=False, default_value=False,
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
  serialized_start=27,
  serialized_end=118,
)


_GRSIM_BALLREPLACEMENT = _descriptor.Descriptor(
  name='grSim_BallReplacement',
  full_name='grSim_BallReplacement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='grSim_BallReplacement.x', index=0,
      number=1, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='grSim_BallReplacement.y', index=1,
      number=2, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vx', full_name='grSim_BallReplacement.vx', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='vy', full_name='grSim_BallReplacement.vy', index=3,
      number=4, type=1, cpp_type=5, label=2,
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
  serialized_start=120,
  serialized_end=189,
)


_GRSIM_REPLACEMENT = _descriptor.Descriptor(
  name='grSim_Replacement',
  full_name='grSim_Replacement',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='ball', full_name='grSim_Replacement.ball', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='robots', full_name='grSim_Replacement.robots', index=1,
      number=2, type=11, cpp_type=10, label=3,
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
  serialized_start=191,
  serialized_end=289,
)

_GRSIM_REPLACEMENT.fields_by_name['ball'].message_type = _GRSIM_BALLREPLACEMENT
_GRSIM_REPLACEMENT.fields_by_name['robots'].message_type = _GRSIM_ROBOTREPLACEMENT
DESCRIPTOR.message_types_by_name['grSim_RobotReplacement'] = _GRSIM_ROBOTREPLACEMENT
DESCRIPTOR.message_types_by_name['grSim_BallReplacement'] = _GRSIM_BALLREPLACEMENT
DESCRIPTOR.message_types_by_name['grSim_Replacement'] = _GRSIM_REPLACEMENT

class grSim_RobotReplacement(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _GRSIM_ROBOTREPLACEMENT

  # @@protoc_insertion_point(class_scope:grSim_RobotReplacement)

class grSim_BallReplacement(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _GRSIM_BALLREPLACEMENT

  # @@protoc_insertion_point(class_scope:grSim_BallReplacement)

class grSim_Replacement(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _GRSIM_REPLACEMENT

  # @@protoc_insertion_point(class_scope:grSim_Replacement)


# @@protoc_insertion_point(module_scope)
