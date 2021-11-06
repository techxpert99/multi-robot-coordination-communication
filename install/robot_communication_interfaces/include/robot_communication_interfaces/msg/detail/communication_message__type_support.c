// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_communication_interfaces/msg/detail/communication_message__rosidl_typesupport_introspection_c.h"
#include "robot_communication_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_communication_interfaces/msg/detail/communication_message__functions.h"
#include "robot_communication_interfaces/msg/detail/communication_message__struct.h"


// Include directives for member types
// Member `message_type`
// Member `robot_id`
// Member `receiver_robot_id`
// Member `suitability_robots`
// Member `suitability_roles`
// Member `teams_id`
// Member `teams_rol`
#include "rosidl_runtime_c/string_functions.h"
// Member `suitability_values`
// Member `teams_tid`
// Member `teams_su`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_communication_interfaces__msg__CommunicationMessage__init(message_memory);
}

void CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_fini_function(void * message_memory)
{
  robot_communication_interfaces__msg__CommunicationMessage__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_message_member_array[12] = {
  {
    "message_type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, message_type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "receiver_robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, receiver_robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "suitability_robots",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, suitability_robots),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "suitability_roles",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, suitability_roles),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "suitability_values",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, suitability_values),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "teams_tid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, teams_tid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "teams_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, teams_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "teams_rol",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, teams_rol),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "teams_su",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, teams_su),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "exchange_number",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces__msg__CommunicationMessage, exchange_number),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_message_members = {
  "robot_communication_interfaces__msg",  // message namespace
  "CommunicationMessage",  // message name
  12,  // number of fields
  sizeof(robot_communication_interfaces__msg__CommunicationMessage),
  CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_message_member_array,  // message members
  CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_message_type_support_handle = {
  0,
  &CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_communication_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, msg, CommunicationMessage)() {
  if (!CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_message_type_support_handle.typesupport_identifier) {
    CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CommunicationMessage__rosidl_typesupport_introspection_c__CommunicationMessage_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
