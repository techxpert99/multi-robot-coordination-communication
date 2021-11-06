// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__BUILDER_HPP_
#define ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__BUILDER_HPP_

#include "robot_communication_interfaces/srv/detail/communication_service__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_communication_interfaces
{

namespace srv
{

namespace builder
{

class Init_CommunicationService_Request_input_message
{
public:
  Init_CommunicationService_Request_input_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_communication_interfaces::srv::CommunicationService_Request input_message(::robot_communication_interfaces::srv::CommunicationService_Request::_input_message_type arg)
  {
    msg_.input_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_communication_interfaces::srv::CommunicationService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_communication_interfaces::srv::CommunicationService_Request>()
{
  return robot_communication_interfaces::srv::builder::Init_CommunicationService_Request_input_message();
}

}  // namespace robot_communication_interfaces


namespace robot_communication_interfaces
{

namespace srv
{

namespace builder
{

class Init_CommunicationService_Response_output_message
{
public:
  Init_CommunicationService_Response_output_message()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::robot_communication_interfaces::srv::CommunicationService_Response output_message(::robot_communication_interfaces::srv::CommunicationService_Response::_output_message_type arg)
  {
    msg_.output_message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_communication_interfaces::srv::CommunicationService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_communication_interfaces::srv::CommunicationService_Response>()
{
  return robot_communication_interfaces::srv::builder::Init_CommunicationService_Response_output_message();
}

}  // namespace robot_communication_interfaces

#endif  // ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__BUILDER_HPP_
