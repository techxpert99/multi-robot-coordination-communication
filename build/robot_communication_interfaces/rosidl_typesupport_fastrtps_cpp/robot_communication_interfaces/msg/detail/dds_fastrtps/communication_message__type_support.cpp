// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice
#include "robot_communication_interfaces/msg/detail/communication_message__rosidl_typesupport_fastrtps_cpp.hpp"
#include "robot_communication_interfaces/msg/detail/communication_message__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace robot_communication_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_communication_interfaces
cdr_serialize(
  const robot_communication_interfaces::msg::CommunicationMessage & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: message_type
  cdr << ros_message.message_type;
  // Member: robot_id
  cdr << ros_message.robot_id;
  // Member: position
  {
    cdr << ros_message.position;
  }
  // Member: receiver_robot_id
  cdr << ros_message.receiver_robot_id;
  // Member: suitability_robots
  {
    size_t size = ros_message.suitability_robots.size();
    if (size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << ros_message.suitability_robots[i];
    }
  }
  // Member: suitability_roles
  {
    size_t size = ros_message.suitability_roles.size();
    if (size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << ros_message.suitability_roles[i];
    }
  }
  // Member: suitability_values
  {
    size_t size = ros_message.suitability_values.size();
    if (size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << ros_message.suitability_values[i];
    }
  }
  // Member: teams_tid
  {
    size_t size = ros_message.teams_tid.size();
    if (size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << ros_message.teams_tid[i];
    }
  }
  // Member: teams_id
  {
    size_t size = ros_message.teams_id.size();
    if (size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << ros_message.teams_id[i];
    }
  }
  // Member: teams_rol
  {
    size_t size = ros_message.teams_rol.size();
    if (size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << ros_message.teams_rol[i];
    }
  }
  // Member: teams_su
  {
    size_t size = ros_message.teams_su.size();
    if (size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }
    cdr << static_cast<uint32_t>(size);
    for (size_t i = 0; i < size; i++) {
      cdr << ros_message.teams_su[i];
    }
  }
  // Member: exchange_number
  cdr << ros_message.exchange_number;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_communication_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  robot_communication_interfaces::msg::CommunicationMessage & ros_message)
{
  // Member: message_type
  cdr >> ros_message.message_type;

  // Member: robot_id
  cdr >> ros_message.robot_id;

  // Member: position
  {
    cdr >> ros_message.position;
  }

  // Member: receiver_robot_id
  cdr >> ros_message.receiver_robot_id;

  // Member: suitability_robots
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.suitability_robots.resize(size);
    for (size_t i = 0; i < size; i++) {
      cdr >> ros_message.suitability_robots[i];
    }
  }

  // Member: suitability_roles
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.suitability_roles.resize(size);
    for (size_t i = 0; i < size; i++) {
      cdr >> ros_message.suitability_roles[i];
    }
  }

  // Member: suitability_values
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.suitability_values.resize(size);
    for (size_t i = 0; i < size; i++) {
      cdr >> ros_message.suitability_values[i];
    }
  }

  // Member: teams_tid
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.teams_tid.resize(size);
    for (size_t i = 0; i < size; i++) {
      cdr >> ros_message.teams_tid[i];
    }
  }

  // Member: teams_id
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.teams_id.resize(size);
    for (size_t i = 0; i < size; i++) {
      cdr >> ros_message.teams_id[i];
    }
  }

  // Member: teams_rol
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.teams_rol.resize(size);
    for (size_t i = 0; i < size; i++) {
      cdr >> ros_message.teams_rol[i];
    }
  }

  // Member: teams_su
  {
    uint32_t cdrSize;
    cdr >> cdrSize;
    size_t size = static_cast<size_t>(cdrSize);
    ros_message.teams_su.resize(size);
    for (size_t i = 0; i < size; i++) {
      cdr >> ros_message.teams_su[i];
    }
  }

  // Member: exchange_number
  cdr >> ros_message.exchange_number;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_communication_interfaces
get_serialized_size(
  const robot_communication_interfaces::msg::CommunicationMessage & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: message_type
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.message_type.size() + 1);
  // Member: robot_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.robot_id.size() + 1);
  // Member: position
  {
    size_t array_size = 3;
    size_t item_size = sizeof(ros_message.position[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: receiver_robot_id
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.receiver_robot_id.size() + 1);
  // Member: suitability_robots
  {
    size_t array_size = ros_message.suitability_robots.size();
    if (array_size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.suitability_robots[index].size() + 1);
    }
  }
  // Member: suitability_roles
  {
    size_t array_size = ros_message.suitability_roles.size();
    if (array_size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.suitability_roles[index].size() + 1);
    }
  }
  // Member: suitability_values
  {
    size_t array_size = ros_message.suitability_values.size();
    if (array_size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.suitability_values[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: teams_tid
  {
    size_t array_size = ros_message.teams_tid.size();
    if (array_size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.teams_tid[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: teams_id
  {
    size_t array_size = ros_message.teams_id.size();
    if (array_size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.teams_id[index].size() + 1);
    }
  }
  // Member: teams_rol
  {
    size_t array_size = ros_message.teams_rol.size();
    if (array_size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        (ros_message.teams_rol[index].size() + 1);
    }
  }
  // Member: teams_su
  {
    size_t array_size = ros_message.teams_su.size();
    if (array_size > 1024) {
      throw std::runtime_error("array size exceeds upper bound");
    }

    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);
    size_t item_size = sizeof(ros_message.teams_su[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: exchange_number
  {
    size_t item_size = sizeof(ros_message.exchange_number);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_robot_communication_interfaces
max_serialized_size_CommunicationMessage(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: message_type
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: robot_id
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: position
  {
    size_t array_size = 3;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: receiver_robot_id
  {
    size_t array_size = 1;

    full_bounded = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: suitability_robots
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

  // Member: suitability_roles
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

  // Member: suitability_values
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: teams_tid
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: teams_id
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

  // Member: teams_rol
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

  // Member: teams_su
  {
    size_t array_size = 1024;
    full_bounded = false;
    current_alignment += padding +
      eprosima::fastcdr::Cdr::alignment(current_alignment, padding);

    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: exchange_number
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _CommunicationMessage__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const robot_communication_interfaces::msg::CommunicationMessage *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _CommunicationMessage__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<robot_communication_interfaces::msg::CommunicationMessage *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _CommunicationMessage__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const robot_communication_interfaces::msg::CommunicationMessage *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _CommunicationMessage__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_CommunicationMessage(full_bounded, 0);
}

static message_type_support_callbacks_t _CommunicationMessage__callbacks = {
  "robot_communication_interfaces::msg",
  "CommunicationMessage",
  _CommunicationMessage__cdr_serialize,
  _CommunicationMessage__cdr_deserialize,
  _CommunicationMessage__get_serialized_size,
  _CommunicationMessage__max_serialized_size
};

static rosidl_message_type_support_t _CommunicationMessage__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_CommunicationMessage__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace robot_communication_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_robot_communication_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_communication_interfaces::msg::CommunicationMessage>()
{
  return &robot_communication_interfaces::msg::typesupport_fastrtps_cpp::_CommunicationMessage__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, robot_communication_interfaces, msg, CommunicationMessage)() {
  return &robot_communication_interfaces::msg::typesupport_fastrtps_cpp::_CommunicationMessage__handle;
}

#ifdef __cplusplus
}
#endif
