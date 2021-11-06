// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice
#include "robot_communication_interfaces/msg/detail/communication_message__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


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

bool
robot_communication_interfaces__msg__CommunicationMessage__init(robot_communication_interfaces__msg__CommunicationMessage * msg)
{
  if (!msg) {
    return false;
  }
  // message_type
  if (!rosidl_runtime_c__String__init(&msg->message_type)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // robot_id
  if (!rosidl_runtime_c__String__init(&msg->robot_id)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // position
  // receiver_robot_id
  if (!rosidl_runtime_c__String__init(&msg->receiver_robot_id)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // suitability_robots
  if (!rosidl_runtime_c__String__Sequence__init(&msg->suitability_robots, 0)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // suitability_roles
  if (!rosidl_runtime_c__String__Sequence__init(&msg->suitability_roles, 0)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // suitability_values
  if (!rosidl_runtime_c__float__Sequence__init(&msg->suitability_values, 0)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // teams_tid
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->teams_tid, 0)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // teams_id
  if (!rosidl_runtime_c__String__Sequence__init(&msg->teams_id, 0)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // teams_rol
  if (!rosidl_runtime_c__String__Sequence__init(&msg->teams_rol, 0)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // teams_su
  if (!rosidl_runtime_c__float__Sequence__init(&msg->teams_su, 0)) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
    return false;
  }
  // exchange_number
  return true;
}

void
robot_communication_interfaces__msg__CommunicationMessage__fini(robot_communication_interfaces__msg__CommunicationMessage * msg)
{
  if (!msg) {
    return;
  }
  // message_type
  rosidl_runtime_c__String__fini(&msg->message_type);
  // robot_id
  rosidl_runtime_c__String__fini(&msg->robot_id);
  // position
  // receiver_robot_id
  rosidl_runtime_c__String__fini(&msg->receiver_robot_id);
  // suitability_robots
  rosidl_runtime_c__String__Sequence__fini(&msg->suitability_robots);
  // suitability_roles
  rosidl_runtime_c__String__Sequence__fini(&msg->suitability_roles);
  // suitability_values
  rosidl_runtime_c__float__Sequence__fini(&msg->suitability_values);
  // teams_tid
  rosidl_runtime_c__uint32__Sequence__fini(&msg->teams_tid);
  // teams_id
  rosidl_runtime_c__String__Sequence__fini(&msg->teams_id);
  // teams_rol
  rosidl_runtime_c__String__Sequence__fini(&msg->teams_rol);
  // teams_su
  rosidl_runtime_c__float__Sequence__fini(&msg->teams_su);
  // exchange_number
}

robot_communication_interfaces__msg__CommunicationMessage *
robot_communication_interfaces__msg__CommunicationMessage__create()
{
  robot_communication_interfaces__msg__CommunicationMessage * msg = (robot_communication_interfaces__msg__CommunicationMessage *)malloc(sizeof(robot_communication_interfaces__msg__CommunicationMessage));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_communication_interfaces__msg__CommunicationMessage));
  bool success = robot_communication_interfaces__msg__CommunicationMessage__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
robot_communication_interfaces__msg__CommunicationMessage__destroy(robot_communication_interfaces__msg__CommunicationMessage * msg)
{
  if (msg) {
    robot_communication_interfaces__msg__CommunicationMessage__fini(msg);
  }
  free(msg);
}


bool
robot_communication_interfaces__msg__CommunicationMessage__Sequence__init(robot_communication_interfaces__msg__CommunicationMessage__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  robot_communication_interfaces__msg__CommunicationMessage * data = NULL;
  if (size) {
    data = (robot_communication_interfaces__msg__CommunicationMessage *)calloc(size, sizeof(robot_communication_interfaces__msg__CommunicationMessage));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_communication_interfaces__msg__CommunicationMessage__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_communication_interfaces__msg__CommunicationMessage__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_communication_interfaces__msg__CommunicationMessage__Sequence__fini(robot_communication_interfaces__msg__CommunicationMessage__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_communication_interfaces__msg__CommunicationMessage__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_communication_interfaces__msg__CommunicationMessage__Sequence *
robot_communication_interfaces__msg__CommunicationMessage__Sequence__create(size_t size)
{
  robot_communication_interfaces__msg__CommunicationMessage__Sequence * array = (robot_communication_interfaces__msg__CommunicationMessage__Sequence *)malloc(sizeof(robot_communication_interfaces__msg__CommunicationMessage__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = robot_communication_interfaces__msg__CommunicationMessage__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
robot_communication_interfaces__msg__CommunicationMessage__Sequence__destroy(robot_communication_interfaces__msg__CommunicationMessage__Sequence * array)
{
  if (array) {
    robot_communication_interfaces__msg__CommunicationMessage__Sequence__fini(array);
  }
  free(array);
}
