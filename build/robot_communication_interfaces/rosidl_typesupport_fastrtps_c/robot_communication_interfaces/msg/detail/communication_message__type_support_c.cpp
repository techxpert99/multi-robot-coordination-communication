// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice
#include "robot_communication_interfaces/msg/detail/communication_message__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "robot_communication_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_communication_interfaces/msg/detail/communication_message__struct.h"
#include "robot_communication_interfaces/msg/detail/communication_message__functions.h"
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

#include "rosidl_runtime_c/primitives_sequence.h"  // suitability_values, teams_su, teams_tid
#include "rosidl_runtime_c/primitives_sequence_functions.h"  // suitability_values, teams_su, teams_tid
#include "rosidl_runtime_c/string.h"  // message_type, receiver_robot_id, robot_id, suitability_robots, suitability_roles, teams_id, teams_rol
#include "rosidl_runtime_c/string_functions.h"  // message_type, receiver_robot_id, robot_id, suitability_robots, suitability_roles, teams_id, teams_rol

// forward declare type support functions


using _CommunicationMessage__ros_msg_type = robot_communication_interfaces__msg__CommunicationMessage;

static bool _CommunicationMessage__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _CommunicationMessage__ros_msg_type * ros_message = static_cast<const _CommunicationMessage__ros_msg_type *>(untyped_ros_message);
  // Field name: message_type
  {
    const rosidl_runtime_c__String * str = &ros_message->message_type;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: robot_id
  {
    const rosidl_runtime_c__String * str = &ros_message->robot_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: position
  {
    size_t size = 3;
    auto array_ptr = ros_message->position;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: receiver_robot_id
  {
    const rosidl_runtime_c__String * str = &ros_message->receiver_robot_id;
    if (str->capacity == 0 || str->capacity <= str->size) {
      fprintf(stderr, "string capacity not greater than size\n");
      return false;
    }
    if (str->data[str->size] != '\0') {
      fprintf(stderr, "string not null-terminated\n");
      return false;
    }
    cdr << str->data;
  }

  // Field name: suitability_robots
  {
    size_t size = ros_message->suitability_robots.size;
    auto array_ptr = ros_message->suitability_robots.data;
    if (size > 1024) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: suitability_roles
  {
    size_t size = ros_message->suitability_roles.size;
    auto array_ptr = ros_message->suitability_roles.data;
    if (size > 1024) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: suitability_values
  {
    size_t size = ros_message->suitability_values.size;
    auto array_ptr = ros_message->suitability_values.data;
    if (size > 1024) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: teams_tid
  {
    size_t size = ros_message->teams_tid.size;
    auto array_ptr = ros_message->teams_tid.data;
    if (size > 1024) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: teams_id
  {
    size_t size = ros_message->teams_id.size;
    auto array_ptr = ros_message->teams_id.data;
    if (size > 1024) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: teams_rol
  {
    size_t size = ros_message->teams_rol.size;
    auto array_ptr = ros_message->teams_rol.data;
    if (size > 1024) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; ++i) {
      const rosidl_runtime_c__String * str = &array_ptr[i];
      if (str->capacity == 0 || str->capacity <= str->size) {
        fprintf(stderr, "string capacity not greater than size\n");
        return false;
      }
      if (str->data[str->size] != '\0') {
        fprintf(stderr, "string not null-terminated\n");
        return false;
      }
      cdr << str->data;
    }
  }

  // Field name: teams_su
  {
    size_t size = ros_message->teams_su.size;
    auto array_ptr = ros_message->teams_su.data;
    if (size > 1024) {
      fprintf(stderr, "array size exceeds upper bound\n");
      return false;
    }
    cdr << static_cast<uint32_t>(size);
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: exchange_number
  {
    cdr << ros_message->exchange_number;
  }

  return true;
}

static bool _CommunicationMessage__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _CommunicationMessage__ros_msg_type * ros_message = static_cast<_CommunicationMessage__ros_msg_type *>(untyped_ros_message);
  // Field name: message_type
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->message_type.data) {
      rosidl_runtime_c__String__init(&ros_message->message_type);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->message_type,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'message_type'\n");
      return false;
    }
  }

  // Field name: robot_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->robot_id.data) {
      rosidl_runtime_c__String__init(&ros_message->robot_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->robot_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'robot_id'\n");
      return false;
    }
  }

  // Field name: position
  {
    size_t size = 3;
    auto array_ptr = ros_message->position;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: receiver_robot_id
  {
    std::string tmp;
    cdr >> tmp;
    if (!ros_message->receiver_robot_id.data) {
      rosidl_runtime_c__String__init(&ros_message->receiver_robot_id);
    }
    bool succeeded = rosidl_runtime_c__String__assign(
      &ros_message->receiver_robot_id,
      tmp.c_str());
    if (!succeeded) {
      fprintf(stderr, "failed to assign string into field 'receiver_robot_id'\n");
      return false;
    }
  }

  // Field name: suitability_robots
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->suitability_robots.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->suitability_robots);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->suitability_robots, size)) {
      return "failed to create array for field 'suitability_robots'";
    }
    auto array_ptr = ros_message->suitability_robots.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'suitability_robots'\n");
        return false;
      }
    }
  }

  // Field name: suitability_roles
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->suitability_roles.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->suitability_roles);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->suitability_roles, size)) {
      return "failed to create array for field 'suitability_roles'";
    }
    auto array_ptr = ros_message->suitability_roles.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'suitability_roles'\n");
        return false;
      }
    }
  }

  // Field name: suitability_values
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->suitability_values.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->suitability_values);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->suitability_values, size)) {
      return "failed to create array for field 'suitability_values'";
    }
    auto array_ptr = ros_message->suitability_values.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: teams_tid
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->teams_tid.data) {
      rosidl_runtime_c__uint32__Sequence__fini(&ros_message->teams_tid);
    }
    if (!rosidl_runtime_c__uint32__Sequence__init(&ros_message->teams_tid, size)) {
      return "failed to create array for field 'teams_tid'";
    }
    auto array_ptr = ros_message->teams_tid.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: teams_id
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->teams_id.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->teams_id);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->teams_id, size)) {
      return "failed to create array for field 'teams_id'";
    }
    auto array_ptr = ros_message->teams_id.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'teams_id'\n");
        return false;
      }
    }
  }

  // Field name: teams_rol
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->teams_rol.data) {
      rosidl_runtime_c__String__Sequence__fini(&ros_message->teams_rol);
    }
    if (!rosidl_runtime_c__String__Sequence__init(&ros_message->teams_rol, size)) {
      return "failed to create array for field 'teams_rol'";
    }
    auto array_ptr = ros_message->teams_rol.data;
    for (size_t i = 0; i < size; ++i) {
      std::string tmp;
      cdr >> tmp;
      auto & ros_i = array_ptr[i];
      if (!ros_i.data) {
        rosidl_runtime_c__String__init(&ros_i);
      }
      bool succeeded = rosidl_runtime_c__String__assign(
        &ros_i,
        tmp.c_str());
      if (!succeeded) {
        fprintf(stderr, "failed to assign string into field 'teams_rol'\n");
        return false;
      }
    }
  }

  // Field name: teams_su
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    if (ros_message->teams_su.data) {
      rosidl_runtime_c__float__Sequence__fini(&ros_message->teams_su);
    }
    if (!rosidl_runtime_c__float__Sequence__init(&ros_message->teams_su, size)) {
      return "failed to create array for field 'teams_su'";
    }
    auto array_ptr = ros_message->teams_su.data;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: exchange_number
  {
    cdr >> ros_message->exchange_number;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_communication_interfaces
size_t get_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _CommunicationMessage__ros_msg_type * ros_message = static_cast<const _CommunicationMessage__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name message_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->message_type.size + 1);
  // field.name robot_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->robot_id.size + 1);
  // field.name position
  {
    size_t array_size = 3;
    auto array_ptr = ros_message->position;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name receiver_robot_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message->receiver_robot_id.size + 1);
  // field.name suitability_robots
  {
    size_t array_size = ros_message->suitability_robots.size;
    auto array_ptr = ros_message->suitability_robots.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name suitability_roles
  {
    size_t array_size = ros_message->suitability_roles.size;
    auto array_ptr = ros_message->suitability_roles.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name suitability_values
  {
    size_t array_size = ros_message->suitability_values.size;
    auto array_ptr = ros_message->suitability_values.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name teams_tid
  {
    size_t array_size = ros_message->teams_tid.size;
    auto array_ptr = ros_message->teams_tid.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name teams_id
  {
    size_t array_size = ros_message->teams_id.size;
    auto array_ptr = ros_message->teams_id.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name teams_rol
  {
    size_t array_size = ros_message->teams_rol.size;
    auto array_ptr = ros_message->teams_rol.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (array_ptr[index].size + 1);
    }
  }
  // field.name teams_su
  {
    size_t array_size = ros_message->teams_su.size;
    auto array_ptr = ros_message->teams_su.data;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name exchange_number
  {
    size_t item_size = sizeof(ros_message->exchange_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _CommunicationMessage__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_communication_interfaces
size_t max_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: message_type
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: robot_id
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: position
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: receiver_robot_id
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: suitability_robots
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: suitability_roles
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: suitability_values
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: teams_tid
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: teams_id
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: teams_rol
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }
  // member: teams_su
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: exchange_number
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _CommunicationMessage__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_robot_communication_interfaces__msg__CommunicationMessage(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_CommunicationMessage = {
  "robot_communication_interfaces::msg",
  "CommunicationMessage",
  _CommunicationMessage__cdr_serialize,
  _CommunicationMessage__cdr_deserialize,
  _CommunicationMessage__get_serialized_size,
  _CommunicationMessage__max_serialized_size
};

static rosidl_message_type_support_t _CommunicationMessage__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_CommunicationMessage,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_communication_interfaces, msg, CommunicationMessage)() {
  return &_CommunicationMessage__type_support;
}

#if defined(__cplusplus)
}
#endif
