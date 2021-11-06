# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_communication_interfaces:msg/CommunicationMessage.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'suitability_values'
# Member 'teams_tid'
# Member 'teams_su'
import array  # noqa: E402, I100

# Member 'position'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CommunicationMessage(type):
    """Metaclass of message 'CommunicationMessage'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_communication_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_communication_interfaces.msg.CommunicationMessage')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__communication_message
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__communication_message
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__communication_message
            cls._TYPE_SUPPORT = module.type_support_msg__msg__communication_message
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__communication_message

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommunicationMessage(metaclass=Metaclass_CommunicationMessage):
    """Message class 'CommunicationMessage'."""

    __slots__ = [
        '_message_type',
        '_robot_id',
        '_position',
        '_receiver_robot_id',
        '_suitability_robots',
        '_suitability_roles',
        '_suitability_values',
        '_teams_tid',
        '_teams_id',
        '_teams_rol',
        '_teams_su',
        '_exchange_number',
    ]

    _fields_and_field_types = {
        'message_type': 'string',
        'robot_id': 'string',
        'position': 'double[3]',
        'receiver_robot_id': 'string',
        'suitability_robots': 'sequence<string, 1024>',
        'suitability_roles': 'sequence<string, 1024>',
        'suitability_values': 'sequence<float, 1024>',
        'teams_tid': 'sequence<uint32, 1024>',
        'teams_id': 'sequence<string, 1024>',
        'teams_rol': 'sequence<string, 1024>',
        'teams_su': 'sequence<float, 1024>',
        'exchange_number': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('double'), 3),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.UnboundedString(), 1024),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.UnboundedString(), 1024),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.BasicType('float'), 1024),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.BasicType('uint32'), 1024),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.UnboundedString(), 1024),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.UnboundedString(), 1024),  # noqa: E501
        rosidl_parser.definition.BoundedSequence(rosidl_parser.definition.BasicType('float'), 1024),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.message_type = kwargs.get('message_type', str())
        self.robot_id = kwargs.get('robot_id', str())
        if 'position' not in kwargs:
            self.position = numpy.zeros(3, dtype=numpy.float64)
        else:
            self.position = numpy.array(kwargs.get('position'), dtype=numpy.float64)
            assert self.position.shape == (3, )
        self.receiver_robot_id = kwargs.get('receiver_robot_id', str())
        self.suitability_robots = kwargs.get('suitability_robots', [])
        self.suitability_roles = kwargs.get('suitability_roles', [])
        self.suitability_values = array.array('f', kwargs.get('suitability_values', []))
        self.teams_tid = array.array('I', kwargs.get('teams_tid', []))
        self.teams_id = kwargs.get('teams_id', [])
        self.teams_rol = kwargs.get('teams_rol', [])
        self.teams_su = array.array('f', kwargs.get('teams_su', []))
        self.exchange_number = kwargs.get('exchange_number', int())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.message_type != other.message_type:
            return False
        if self.robot_id != other.robot_id:
            return False
        if all(self.position != other.position):
            return False
        if self.receiver_robot_id != other.receiver_robot_id:
            return False
        if self.suitability_robots != other.suitability_robots:
            return False
        if self.suitability_roles != other.suitability_roles:
            return False
        if self.suitability_values != other.suitability_values:
            return False
        if self.teams_tid != other.teams_tid:
            return False
        if self.teams_id != other.teams_id:
            return False
        if self.teams_rol != other.teams_rol:
            return False
        if self.teams_su != other.teams_su:
            return False
        if self.exchange_number != other.exchange_number:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def message_type(self):
        """Message field 'message_type'."""
        return self._message_type

    @message_type.setter
    def message_type(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message_type' field must be of type 'str'"
        self._message_type = value

    @property
    def robot_id(self):
        """Message field 'robot_id'."""
        return self._robot_id

    @robot_id.setter
    def robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'robot_id' field must be of type 'str'"
        self._robot_id = value

    @property
    def position(self):
        """Message field 'position'."""
        return self._position

    @position.setter
    def position(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.float64, \
                "The 'position' numpy.ndarray() must have the dtype of 'numpy.float64'"
            assert value.size == 3, \
                "The 'position' numpy.ndarray() must have a size of 3"
            self._position = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) == 3 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'position' field must be a set or sequence with length 3 and each value of type 'float'"
        self._position = numpy.array(value, dtype=numpy.float64)

    @property
    def receiver_robot_id(self):
        """Message field 'receiver_robot_id'."""
        return self._receiver_robot_id

    @receiver_robot_id.setter
    def receiver_robot_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'receiver_robot_id' field must be of type 'str'"
        self._receiver_robot_id = value

    @property
    def suitability_robots(self):
        """Message field 'suitability_robots'."""
        return self._suitability_robots

    @suitability_robots.setter
    def suitability_robots(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1024 and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'suitability_robots' field must be a set or sequence with length <= 1024 and each value of type 'str'"
        self._suitability_robots = value

    @property
    def suitability_roles(self):
        """Message field 'suitability_roles'."""
        return self._suitability_roles

    @suitability_roles.setter
    def suitability_roles(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1024 and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'suitability_roles' field must be a set or sequence with length <= 1024 and each value of type 'str'"
        self._suitability_roles = value

    @property
    def suitability_values(self):
        """Message field 'suitability_values'."""
        return self._suitability_values

    @suitability_values.setter
    def suitability_values(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'suitability_values' array.array() must have the type code of 'f'"
            assert len(value) <= 1024, \
                "The 'suitability_values' array.array() must have a size <= 1024"
            self._suitability_values = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1024 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'suitability_values' field must be a set or sequence with length <= 1024 and each value of type 'float'"
        self._suitability_values = array.array('f', value)

    @property
    def teams_tid(self):
        """Message field 'teams_tid'."""
        return self._teams_tid

    @teams_tid.setter
    def teams_tid(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'I', \
                "The 'teams_tid' array.array() must have the type code of 'I'"
            assert len(value) <= 1024, \
                "The 'teams_tid' array.array() must have a size <= 1024"
            self._teams_tid = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1024 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 4294967296 for val in value)), \
                "The 'teams_tid' field must be a set or sequence with length <= 1024 and each value of type 'int' and each unsigned integer in [0, 4294967295]"
        self._teams_tid = array.array('I', value)

    @property
    def teams_id(self):
        """Message field 'teams_id'."""
        return self._teams_id

    @teams_id.setter
    def teams_id(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1024 and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'teams_id' field must be a set or sequence with length <= 1024 and each value of type 'str'"
        self._teams_id = value

    @property
    def teams_rol(self):
        """Message field 'teams_rol'."""
        return self._teams_rol

    @teams_rol.setter
    def teams_rol(self, value):
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1024 and
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'teams_rol' field must be a set or sequence with length <= 1024 and each value of type 'str'"
        self._teams_rol = value

    @property
    def teams_su(self):
        """Message field 'teams_su'."""
        return self._teams_su

    @teams_su.setter
    def teams_su(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'teams_su' array.array() must have the type code of 'f'"
            assert len(value) <= 1024, \
                "The 'teams_su' array.array() must have a size <= 1024"
            self._teams_su = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 len(value) <= 1024 and
                 all(isinstance(v, float) for v in value) and
                 True), \
                "The 'teams_su' field must be a set or sequence with length <= 1024 and each value of type 'float'"
        self._teams_su = array.array('f', value)

    @property
    def exchange_number(self):
        """Message field 'exchange_number'."""
        return self._exchange_number

    @exchange_number.setter
    def exchange_number(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'exchange_number' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'exchange_number' field must be an unsigned integer in [0, 18446744073709551615]"
        self._exchange_number = value
