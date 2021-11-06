// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__STRUCT_H_
#define ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'input_message'
#include "robot_communication_interfaces/msg/detail/communication_message__struct.h"

// Struct defined in srv/CommunicationService in the package robot_communication_interfaces.
typedef struct robot_communication_interfaces__srv__CommunicationService_Request
{
  robot_communication_interfaces__msg__CommunicationMessage input_message;
} robot_communication_interfaces__srv__CommunicationService_Request;

// Struct for a sequence of robot_communication_interfaces__srv__CommunicationService_Request.
typedef struct robot_communication_interfaces__srv__CommunicationService_Request__Sequence
{
  robot_communication_interfaces__srv__CommunicationService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_communication_interfaces__srv__CommunicationService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'output_message'
// already included above
// #include "robot_communication_interfaces/msg/detail/communication_message__struct.h"

// Struct defined in srv/CommunicationService in the package robot_communication_interfaces.
typedef struct robot_communication_interfaces__srv__CommunicationService_Response
{
  robot_communication_interfaces__msg__CommunicationMessage output_message;
} robot_communication_interfaces__srv__CommunicationService_Response;

// Struct for a sequence of robot_communication_interfaces__srv__CommunicationService_Response.
typedef struct robot_communication_interfaces__srv__CommunicationService_Response__Sequence
{
  robot_communication_interfaces__srv__CommunicationService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_communication_interfaces__srv__CommunicationService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__STRUCT_H_
