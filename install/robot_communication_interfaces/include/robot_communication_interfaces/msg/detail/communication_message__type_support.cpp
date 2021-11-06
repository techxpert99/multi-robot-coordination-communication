// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_communication_interfaces/msg/detail/communication_message__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace robot_communication_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void CommunicationMessage_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) robot_communication_interfaces::msg::CommunicationMessage(_init);
}

void CommunicationMessage_fini_function(void * message_memory)
{
  auto typed_message = static_cast<robot_communication_interfaces::msg::CommunicationMessage *>(message_memory);
  typed_message->~CommunicationMessage();
}

size_t size_function__CommunicationMessage__position(const void * untyped_member)
{
  (void)untyped_member;
  return 3;
}

const void * get_const_function__CommunicationMessage__position(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::array<double, 3> *>(untyped_member);
  return &member[index];
}

void * get_function__CommunicationMessage__position(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::array<double, 3> *>(untyped_member);
  return &member[index];
}

size_t size_function__CommunicationMessage__suitability_robots(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CommunicationMessage__suitability_robots(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__CommunicationMessage__suitability_robots(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__CommunicationMessage__suitability_robots(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CommunicationMessage__suitability_roles(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CommunicationMessage__suitability_roles(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__CommunicationMessage__suitability_roles(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__CommunicationMessage__suitability_roles(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CommunicationMessage__suitability_values(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CommunicationMessage__suitability_values(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CommunicationMessage__suitability_values(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__CommunicationMessage__suitability_values(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CommunicationMessage__teams_tid(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CommunicationMessage__teams_tid(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void * get_function__CommunicationMessage__teams_tid(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  return &member[index];
}

void resize_function__CommunicationMessage__teams_tid(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<uint32_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CommunicationMessage__teams_id(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CommunicationMessage__teams_id(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__CommunicationMessage__teams_id(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__CommunicationMessage__teams_id(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CommunicationMessage__teams_rol(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CommunicationMessage__teams_rol(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void * get_function__CommunicationMessage__teams_rol(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<std::string> *>(untyped_member);
  return &member[index];
}

void resize_function__CommunicationMessage__teams_rol(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<std::string> *>(untyped_member);
  member->resize(size);
}

size_t size_function__CommunicationMessage__teams_su(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__CommunicationMessage__teams_su(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__CommunicationMessage__teams_su(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void resize_function__CommunicationMessage__teams_su(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember CommunicationMessage_message_member_array[12] = {
  {
    "message_type",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, message_type),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "robot_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, robot_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    3,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, position),  // bytes offset in struct
    nullptr,  // default value
    size_function__CommunicationMessage__position,  // size() function pointer
    get_const_function__CommunicationMessage__position,  // get_const(index) function pointer
    get_function__CommunicationMessage__position,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "receiver_robot_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, receiver_robot_id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "suitability_robots",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, suitability_robots),  // bytes offset in struct
    nullptr,  // default value
    size_function__CommunicationMessage__suitability_robots,  // size() function pointer
    get_const_function__CommunicationMessage__suitability_robots,  // get_const(index) function pointer
    get_function__CommunicationMessage__suitability_robots,  // get(index) function pointer
    resize_function__CommunicationMessage__suitability_robots  // resize(index) function pointer
  },
  {
    "suitability_roles",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, suitability_roles),  // bytes offset in struct
    nullptr,  // default value
    size_function__CommunicationMessage__suitability_roles,  // size() function pointer
    get_const_function__CommunicationMessage__suitability_roles,  // get_const(index) function pointer
    get_function__CommunicationMessage__suitability_roles,  // get(index) function pointer
    resize_function__CommunicationMessage__suitability_roles  // resize(index) function pointer
  },
  {
    "suitability_values",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, suitability_values),  // bytes offset in struct
    nullptr,  // default value
    size_function__CommunicationMessage__suitability_values,  // size() function pointer
    get_const_function__CommunicationMessage__suitability_values,  // get_const(index) function pointer
    get_function__CommunicationMessage__suitability_values,  // get(index) function pointer
    resize_function__CommunicationMessage__suitability_values  // resize(index) function pointer
  },
  {
    "teams_tid",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, teams_tid),  // bytes offset in struct
    nullptr,  // default value
    size_function__CommunicationMessage__teams_tid,  // size() function pointer
    get_const_function__CommunicationMessage__teams_tid,  // get_const(index) function pointer
    get_function__CommunicationMessage__teams_tid,  // get(index) function pointer
    resize_function__CommunicationMessage__teams_tid  // resize(index) function pointer
  },
  {
    "teams_id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, teams_id),  // bytes offset in struct
    nullptr,  // default value
    size_function__CommunicationMessage__teams_id,  // size() function pointer
    get_const_function__CommunicationMessage__teams_id,  // get_const(index) function pointer
    get_function__CommunicationMessage__teams_id,  // get(index) function pointer
    resize_function__CommunicationMessage__teams_id  // resize(index) function pointer
  },
  {
    "teams_rol",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, teams_rol),  // bytes offset in struct
    nullptr,  // default value
    size_function__CommunicationMessage__teams_rol,  // size() function pointer
    get_const_function__CommunicationMessage__teams_rol,  // get_const(index) function pointer
    get_function__CommunicationMessage__teams_rol,  // get(index) function pointer
    resize_function__CommunicationMessage__teams_rol  // resize(index) function pointer
  },
  {
    "teams_su",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    1024,  // array size
    true,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, teams_su),  // bytes offset in struct
    nullptr,  // default value
    size_function__CommunicationMessage__teams_su,  // size() function pointer
    get_const_function__CommunicationMessage__teams_su,  // get_const(index) function pointer
    get_function__CommunicationMessage__teams_su,  // get(index) function pointer
    resize_function__CommunicationMessage__teams_su  // resize(index) function pointer
  },
  {
    "exchange_number",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces::msg::CommunicationMessage, exchange_number),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers CommunicationMessage_message_members = {
  "robot_communication_interfaces::msg",  // message namespace
  "CommunicationMessage",  // message name
  12,  // number of fields
  sizeof(robot_communication_interfaces::msg::CommunicationMessage),
  CommunicationMessage_message_member_array,  // message members
  CommunicationMessage_init_function,  // function to initialize message memory (memory has to be allocated)
  CommunicationMessage_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t CommunicationMessage_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &CommunicationMessage_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace robot_communication_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<robot_communication_interfaces::msg::CommunicationMessage>()
{
  return &::robot_communication_interfaces::msg::rosidl_typesupport_introspection_cpp::CommunicationMessage_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, robot_communication_interfaces, msg, CommunicationMessage)() {
  return &::robot_communication_interfaces::msg::rosidl_typesupport_introspection_cpp::CommunicationMessage_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
