// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__TRAITS_HPP_
#define ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__TRAITS_HPP_

#include "robot_communication_interfaces/msg/detail/communication_message__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_communication_interfaces::msg::CommunicationMessage>()
{
  return "robot_communication_interfaces::msg::CommunicationMessage";
}

template<>
inline const char * name<robot_communication_interfaces::msg::CommunicationMessage>()
{
  return "robot_communication_interfaces/msg/CommunicationMessage";
}

template<>
struct has_fixed_size<robot_communication_interfaces::msg::CommunicationMessage>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<robot_communication_interfaces::msg::CommunicationMessage>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<robot_communication_interfaces::msg::CommunicationMessage>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__TRAITS_HPP_
