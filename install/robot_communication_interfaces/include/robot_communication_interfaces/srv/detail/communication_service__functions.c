// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice
#include "robot_communication_interfaces/srv/detail/communication_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// Include directives for member types
// Member `input_message`
#include "robot_communication_interfaces/msg/detail/communication_message__functions.h"

bool
robot_communication_interfaces__srv__CommunicationService_Request__init(robot_communication_interfaces__srv__CommunicationService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // input_message
  if (!robot_communication_interfaces__msg__CommunicationMessage__init(&msg->input_message)) {
    robot_communication_interfaces__srv__CommunicationService_Request__fini(msg);
    return false;
  }
  return true;
}

void
robot_communication_interfaces__srv__CommunicationService_Request__fini(robot_communication_interfaces__srv__CommunicationService_Request * msg)
{
  if (!msg) {
    return;
  }
  // input_message
  robot_communication_interfaces__msg__CommunicationMessage__fini(&msg->input_message);
}

robot_communication_interfaces__srv__CommunicationService_Request *
robot_communication_interfaces__srv__CommunicationService_Request__create()
{
  robot_communication_interfaces__srv__CommunicationService_Request * msg = (robot_communication_interfaces__srv__CommunicationService_Request *)malloc(sizeof(robot_communication_interfaces__srv__CommunicationService_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_communication_interfaces__srv__CommunicationService_Request));
  bool success = robot_communication_interfaces__srv__CommunicationService_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
robot_communication_interfaces__srv__CommunicationService_Request__destroy(robot_communication_interfaces__srv__CommunicationService_Request * msg)
{
  if (msg) {
    robot_communication_interfaces__srv__CommunicationService_Request__fini(msg);
  }
  free(msg);
}


bool
robot_communication_interfaces__srv__CommunicationService_Request__Sequence__init(robot_communication_interfaces__srv__CommunicationService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  robot_communication_interfaces__srv__CommunicationService_Request * data = NULL;
  if (size) {
    data = (robot_communication_interfaces__srv__CommunicationService_Request *)calloc(size, sizeof(robot_communication_interfaces__srv__CommunicationService_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_communication_interfaces__srv__CommunicationService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_communication_interfaces__srv__CommunicationService_Request__fini(&data[i - 1]);
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
robot_communication_interfaces__srv__CommunicationService_Request__Sequence__fini(robot_communication_interfaces__srv__CommunicationService_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_communication_interfaces__srv__CommunicationService_Request__fini(&array->data[i]);
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

robot_communication_interfaces__srv__CommunicationService_Request__Sequence *
robot_communication_interfaces__srv__CommunicationService_Request__Sequence__create(size_t size)
{
  robot_communication_interfaces__srv__CommunicationService_Request__Sequence * array = (robot_communication_interfaces__srv__CommunicationService_Request__Sequence *)malloc(sizeof(robot_communication_interfaces__srv__CommunicationService_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = robot_communication_interfaces__srv__CommunicationService_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
robot_communication_interfaces__srv__CommunicationService_Request__Sequence__destroy(robot_communication_interfaces__srv__CommunicationService_Request__Sequence * array)
{
  if (array) {
    robot_communication_interfaces__srv__CommunicationService_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `output_message`
// already included above
// #include "robot_communication_interfaces/msg/detail/communication_message__functions.h"

bool
robot_communication_interfaces__srv__CommunicationService_Response__init(robot_communication_interfaces__srv__CommunicationService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // output_message
  if (!robot_communication_interfaces__msg__CommunicationMessage__init(&msg->output_message)) {
    robot_communication_interfaces__srv__CommunicationService_Response__fini(msg);
    return false;
  }
  return true;
}

void
robot_communication_interfaces__srv__CommunicationService_Response__fini(robot_communication_interfaces__srv__CommunicationService_Response * msg)
{
  if (!msg) {
    return;
  }
  // output_message
  robot_communication_interfaces__msg__CommunicationMessage__fini(&msg->output_message);
}

robot_communication_interfaces__srv__CommunicationService_Response *
robot_communication_interfaces__srv__CommunicationService_Response__create()
{
  robot_communication_interfaces__srv__CommunicationService_Response * msg = (robot_communication_interfaces__srv__CommunicationService_Response *)malloc(sizeof(robot_communication_interfaces__srv__CommunicationService_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_communication_interfaces__srv__CommunicationService_Response));
  bool success = robot_communication_interfaces__srv__CommunicationService_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
robot_communication_interfaces__srv__CommunicationService_Response__destroy(robot_communication_interfaces__srv__CommunicationService_Response * msg)
{
  if (msg) {
    robot_communication_interfaces__srv__CommunicationService_Response__fini(msg);
  }
  free(msg);
}


bool
robot_communication_interfaces__srv__CommunicationService_Response__Sequence__init(robot_communication_interfaces__srv__CommunicationService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  robot_communication_interfaces__srv__CommunicationService_Response * data = NULL;
  if (size) {
    data = (robot_communication_interfaces__srv__CommunicationService_Response *)calloc(size, sizeof(robot_communication_interfaces__srv__CommunicationService_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_communication_interfaces__srv__CommunicationService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_communication_interfaces__srv__CommunicationService_Response__fini(&data[i - 1]);
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
robot_communication_interfaces__srv__CommunicationService_Response__Sequence__fini(robot_communication_interfaces__srv__CommunicationService_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_communication_interfaces__srv__CommunicationService_Response__fini(&array->data[i]);
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

robot_communication_interfaces__srv__CommunicationService_Response__Sequence *
robot_communication_interfaces__srv__CommunicationService_Response__Sequence__create(size_t size)
{
  robot_communication_interfaces__srv__CommunicationService_Response__Sequence * array = (robot_communication_interfaces__srv__CommunicationService_Response__Sequence *)malloc(sizeof(robot_communication_interfaces__srv__CommunicationService_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = robot_communication_interfaces__srv__CommunicationService_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
robot_communication_interfaces__srv__CommunicationService_Response__Sequence__destroy(robot_communication_interfaces__srv__CommunicationService_Response__Sequence * array)
{
  if (array) {
    robot_communication_interfaces__srv__CommunicationService_Response__Sequence__fini(array);
  }
  free(array);
}
