// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__TRAITS_HPP_
#define ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__TRAITS_HPP_

#include "robot_communication_interfaces/srv/detail/communication_service__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'input_message'
#include "robot_communication_interfaces/msg/detail/communication_message__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_communication_interfaces::srv::CommunicationService_Request>()
{
  return "robot_communication_interfaces::srv::CommunicationService_Request";
}

template<>
inline const char * name<robot_communication_interfaces::srv::CommunicationService_Request>()
{
  return "robot_communication_interfaces/srv/CommunicationService_Request";
}

template<>
struct has_fixed_size<robot_communication_interfaces::srv::CommunicationService_Request>
  : std::integral_constant<bool, has_fixed_size<robot_communication_interfaces::msg::CommunicationMessage>::value> {};

template<>
struct has_bounded_size<robot_communication_interfaces::srv::CommunicationService_Request>
  : std::integral_constant<bool, has_bounded_size<robot_communication_interfaces::msg::CommunicationMessage>::value> {};

template<>
struct is_message<robot_communication_interfaces::srv::CommunicationService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'output_message'
// already included above
// #include "robot_communication_interfaces/msg/detail/communication_message__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_communication_interfaces::srv::CommunicationService_Response>()
{
  return "robot_communication_interfaces::srv::CommunicationService_Response";
}

template<>
inline const char * name<robot_communication_interfaces::srv::CommunicationService_Response>()
{
  return "robot_communication_interfaces/srv/CommunicationService_Response";
}

template<>
struct has_fixed_size<robot_communication_interfaces::srv::CommunicationService_Response>
  : std::integral_constant<bool, has_fixed_size<robot_communication_interfaces::msg::CommunicationMessage>::value> {};

template<>
struct has_bounded_size<robot_communication_interfaces::srv::CommunicationService_Response>
  : std::integral_constant<bool, has_bounded_size<robot_communication_interfaces::msg::CommunicationMessage>::value> {};

template<>
struct is_message<robot_communication_interfaces::srv::CommunicationService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<robot_communication_interfaces::srv::CommunicationService>()
{
  return "robot_communication_interfaces::srv::CommunicationService";
}

template<>
inline const char * name<robot_communication_interfaces::srv::CommunicationService>()
{
  return "robot_communication_interfaces/srv/CommunicationService";
}

template<>
struct has_fixed_size<robot_communication_interfaces::srv::CommunicationService>
  : std::integral_constant<
    bool,
    has_fixed_size<robot_communication_interfaces::srv::CommunicationService_Request>::value &&
    has_fixed_size<robot_communication_interfaces::srv::CommunicationService_Response>::value
  >
{
};

template<>
struct has_bounded_size<robot_communication_interfaces::srv::CommunicationService>
  : std::integral_constant<
    bool,
    has_bounded_size<robot_communication_interfaces::srv::CommunicationService_Request>::value &&
    has_bounded_size<robot_communication_interfaces::srv::CommunicationService_Response>::value
  >
{
};

template<>
struct is_service<robot_communication_interfaces::srv::CommunicationService>
  : std::true_type
{
};

template<>
struct is_service_request<robot_communication_interfaces::srv::CommunicationService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<robot_communication_interfaces::srv::CommunicationService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__TRAITS_HPP_
