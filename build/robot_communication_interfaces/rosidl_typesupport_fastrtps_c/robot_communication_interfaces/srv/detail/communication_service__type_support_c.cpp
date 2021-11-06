// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice
#include "robot_communication_interfaces/srv/detail/communication_service__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robot_communication_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_communication_interfaces/srv/detail/communication_service__struct.h"
#include "robot_communication_interfaces/srv/detail/communication_service__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "robot_communication_interfaces/msg/detail/communication_message__functions.h"  // input_message

// forward declare type support functions
size_t get_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_communication_interfaces, msg, CommunicationMessage)();


using _CommunicationService_Request__ros_msg_type = robot_communication_interfaces__srv__CommunicationService_Request;

static bool _CommunicationService_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CommunicationService_Request__ros_msg_type * ros_message = static_cast<const _CommunicationService_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: input_message
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, robot_communication_interfaces, msg, CommunicationMessage
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->input_message, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _CommunicationService_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CommunicationService_Request__ros_msg_type * ros_message = static_cast<_CommunicationService_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: input_message
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, robot_communication_interfaces, msg, CommunicationMessage
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->input_message))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_communication_interfaces
size_t get_serialized_size_robot_communication_interfaces__srv__CommunicationService_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CommunicationService_Request__ros_msg_type * ros_message = static_cast<const _CommunicationService_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name input_message

  current_alignment += get_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
    &(ros_message->input_message), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _CommunicationService_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_communication_interfaces__srv__CommunicationService_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_communication_interfaces
size_t max_serialized_size_robot_communication_interfaces__srv__CommunicationService_Request(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: input_message
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _CommunicationService_Request__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_robot_communication_interfaces__srv__CommunicationService_Request(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_CommunicationService_Request = {
  "robot_communication_interfaces::srv",
  "CommunicationService_Request",
  _CommunicationService_Request__cdr_serialize,
  _CommunicationService_Request__cdr_deserialize,
  _CommunicationService_Request__get_serialized_size,
  _CommunicationService_Request__max_serialized_size
};

static rosidl_message_type_support_t _CommunicationService_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CommunicationService_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_communication_interfaces, srv, CommunicationService_Request)() {
  return &_CommunicationService_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "robot_communication_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "robot_communication_interfaces/srv/detail/communication_service__struct.h"
// already included above
// #include "robot_communication_interfaces/srv/detail/communication_service__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

// already included above
// #include "robot_communication_interfaces/msg/detail/communication_message__functions.h"  // output_message

// forward declare type support functions
size_t get_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
  bool & full_bounded,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_communication_interfaces, msg, CommunicationMessage)();


using _CommunicationService_Response__ros_msg_type = robot_communication_interfaces__srv__CommunicationService_Response;

static bool _CommunicationService_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CommunicationService_Response__ros_msg_type * ros_message = static_cast<const _CommunicationService_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: output_message
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, robot_communication_interfaces, msg, CommunicationMessage
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->output_message, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _CommunicationService_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CommunicationService_Response__ros_msg_type * ros_message = static_cast<_CommunicationService_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: output_message
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, robot_communication_interfaces, msg, CommunicationMessage
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->output_message))
    {
      return false;
    }
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_communication_interfaces
size_t get_serialized_size_robot_communication_interfaces__srv__CommunicationService_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CommunicationService_Response__ros_msg_type * ros_message = static_cast<const _CommunicationService_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name output_message

  current_alignment += get_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
    &(ros_message->output_message), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _CommunicationService_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_communication_interfaces__srv__CommunicationService_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_communication_interfaces
size_t max_serialized_size_robot_communication_interfaces__srv__CommunicationService_Response(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: output_message
  {
    size_t array_size = 1;


    for (size_t index = 0; index < array_size; ++index) {
      current_alignment +=
        max_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
        full_bounded, current_alignment);
    }
  }

  return current_alignment - initial_alignment;
}

static size_t _CommunicationService_Response__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_robot_communication_interfaces__srv__CommunicationService_Response(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_CommunicationService_Response = {
  "robot_communication_interfaces::srv",
  "CommunicationService_Response",
  _CommunicationService_Response__cdr_serialize,
  _CommunicationService_Response__cdr_deserialize,
  _CommunicationService_Response__get_serialized_size,
  _CommunicationService_Response__max_serialized_size
};

static rosidl_message_type_support_t _CommunicationService_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CommunicationService_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_communication_interfaces, srv, CommunicationService_Response)() {
  return &_CommunicationService_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "robot_communication_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_communication_interfaces/srv/communication_service.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t CommunicationService__callbacks = {
  "robot_communication_interfaces::srv",
  "CommunicationService",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_communication_interfaces, srv, CommunicationService_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_communication_interfaces, srv, CommunicationService_Response)(),
};

static rosidl_service_type_support_t CommunicationService__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &CommunicationService__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_communication_interfaces, srv, CommunicationService)() {
  return &CommunicationService__handle;
}

#if defined(__cplusplus)
}
#endif
