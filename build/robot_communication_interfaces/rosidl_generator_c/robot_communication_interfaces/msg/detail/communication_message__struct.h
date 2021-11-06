// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__STRUCT_H_
#define ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'message_type'
// Member 'robot_id'
// Member 'receiver_robot_id'
// Member 'suitability_robots'
// Member 'suitability_roles'
// Member 'teams_id'
// Member 'teams_rol'
#include "rosidl_runtime_c/string.h"
// Member 'suitability_values'
// Member 'teams_tid'
// Member 'teams_su'
#include "rosidl_runtime_c/primitives_sequence.h"

// constants for array fields with an upper bound
// suitability_robots
enum
{
  robot_communication_interfaces__msg__CommunicationMessage__suitability_robots__MAX_SIZE = 1024
};
// suitability_roles
enum
{
  robot_communication_interfaces__msg__CommunicationMessage__suitability_roles__MAX_SIZE = 1024
};
// suitability_values
enum
{
  robot_communication_interfaces__msg__CommunicationMessage__suitability_values__MAX_SIZE = 1024
};
// teams_tid
enum
{
  robot_communication_interfaces__msg__CommunicationMessage__teams_tid__MAX_SIZE = 1024
};
// teams_id
enum
{
  robot_communication_interfaces__msg__CommunicationMessage__teams_id__MAX_SIZE = 1024
};
// teams_rol
enum
{
  robot_communication_interfaces__msg__CommunicationMessage__teams_rol__MAX_SIZE = 1024
};
// teams_su
enum
{
  robot_communication_interfaces__msg__CommunicationMessage__teams_su__MAX_SIZE = 1024
};

// Struct defined in msg/CommunicationMessage in the package robot_communication_interfaces.
typedef struct robot_communication_interfaces__msg__CommunicationMessage
{
  rosidl_runtime_c__String message_type;
  rosidl_runtime_c__String robot_id;
  double position[3];
  rosidl_runtime_c__String receiver_robot_id;
  rosidl_runtime_c__String__Sequence suitability_robots;
  rosidl_runtime_c__String__Sequence suitability_roles;
  rosidl_runtime_c__float__Sequence suitability_values;
  rosidl_runtime_c__uint32__Sequence teams_tid;
  rosidl_runtime_c__String__Sequence teams_id;
  rosidl_runtime_c__String__Sequence teams_rol;
  rosidl_runtime_c__float__Sequence teams_su;
  uint64_t exchange_number;
} robot_communication_interfaces__msg__CommunicationMessage;

// Struct for a sequence of robot_communication_interfaces__msg__CommunicationMessage.
typedef struct robot_communication_interfaces__msg__CommunicationMessage__Sequence
{
  robot_communication_interfaces__msg__CommunicationMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_communication_interfaces__msg__CommunicationMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__STRUCT_H_
