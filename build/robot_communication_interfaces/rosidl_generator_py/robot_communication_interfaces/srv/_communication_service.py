# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_communication_interfaces:srv/CommunicationService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CommunicationService_Request(type):
    """Metaclass of message 'CommunicationService_Request'."""

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
                'robot_communication_interfaces.srv.CommunicationService_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__communication_service__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__communication_service__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__communication_service__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__communication_service__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__communication_service__request

            from robot_communication_interfaces.msg import CommunicationMessage
            if CommunicationMessage.__class__._TYPE_SUPPORT is None:
                CommunicationMessage.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommunicationService_Request(metaclass=Metaclass_CommunicationService_Request):
    """Message class 'CommunicationService_Request'."""

    __slots__ = [
        '_input_message',
    ]

    _fields_and_field_types = {
        'input_message': 'robot_communication_interfaces/CommunicationMessage',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['robot_communication_interfaces', 'msg'], 'CommunicationMessage'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from robot_communication_interfaces.msg import CommunicationMessage
        self.input_message = kwargs.get('input_message', CommunicationMessage())

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
        if self.input_message != other.input_message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def input_message(self):
        """Message field 'input_message'."""
        return self._input_message

    @input_message.setter
    def input_message(self, value):
        if __debug__:
            from robot_communication_interfaces.msg import CommunicationMessage
            assert \
                isinstance(value, CommunicationMessage), \
                "The 'input_message' field must be a sub message of type 'CommunicationMessage'"
        self._input_message = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_CommunicationService_Response(type):
    """Metaclass of message 'CommunicationService_Response'."""

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
                'robot_communication_interfaces.srv.CommunicationService_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__communication_service__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__communication_service__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__communication_service__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__communication_service__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__communication_service__response

            from robot_communication_interfaces.msg import CommunicationMessage
            if CommunicationMessage.__class__._TYPE_SUPPORT is None:
                CommunicationMessage.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CommunicationService_Response(metaclass=Metaclass_CommunicationService_Response):
    """Message class 'CommunicationService_Response'."""

    __slots__ = [
        '_output_message',
    ]

    _fields_and_field_types = {
        'output_message': 'robot_communication_interfaces/CommunicationMessage',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['robot_communication_interfaces', 'msg'], 'CommunicationMessage'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from robot_communication_interfaces.msg import CommunicationMessage
        self.output_message = kwargs.get('output_message', CommunicationMessage())

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
        if self.output_message != other.output_message:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
    def output_message(self):
        """Message field 'output_message'."""
        return self._output_message

    @output_message.setter
    def output_message(self, value):
        if __debug__:
            from robot_communication_interfaces.msg import CommunicationMessage
            assert \
                isinstance(value, CommunicationMessage), \
                "The 'output_message' field must be a sub message of type 'CommunicationMessage'"
        self._output_message = value


class Metaclass_CommunicationService(type):
    """Metaclass of service 'CommunicationService'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_communication_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_communication_interfaces.srv.CommunicationService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__communication_service

            from robot_communication_interfaces.srv import _communication_service
            if _communication_service.Metaclass_CommunicationService_Request._TYPE_SUPPORT is None:
                _communication_service.Metaclass_CommunicationService_Request.__import_type_support__()
            if _communication_service.Metaclass_CommunicationService_Response._TYPE_SUPPORT is None:
                _communication_service.Metaclass_CommunicationService_Response.__import_type_support__()


class CommunicationService(metaclass=Metaclass_CommunicationService):
    from robot_communication_interfaces.srv._communication_service import CommunicationService_Request as Request
    from robot_communication_interfaces.srv._communication_service import CommunicationService_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
