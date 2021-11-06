// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__BUILDER_HPP_
#define ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__BUILDER_HPP_

#include "robot_communication_interfaces/msg/detail/communication_message__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace robot_communication_interfaces
{

namespace msg
{

namespace builder
{

class Init_CommunicationMessage_exchange_number
{
public:
  explicit Init_CommunicationMessage_exchange_number(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  ::robot_communication_interfaces::msg::CommunicationMessage exchange_number(::robot_communication_interfaces::msg::CommunicationMessage::_exchange_number_type arg)
  {
    msg_.exchange_number = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_teams_su
{
public:
  explicit Init_CommunicationMessage_teams_su(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_exchange_number teams_su(::robot_communication_interfaces::msg::CommunicationMessage::_teams_su_type arg)
  {
    msg_.teams_su = std::move(arg);
    return Init_CommunicationMessage_exchange_number(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_teams_rol
{
public:
  explicit Init_CommunicationMessage_teams_rol(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_teams_su teams_rol(::robot_communication_interfaces::msg::CommunicationMessage::_teams_rol_type arg)
  {
    msg_.teams_rol = std::move(arg);
    return Init_CommunicationMessage_teams_su(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_teams_id
{
public:
  explicit Init_CommunicationMessage_teams_id(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_teams_rol teams_id(::robot_communication_interfaces::msg::CommunicationMessage::_teams_id_type arg)
  {
    msg_.teams_id = std::move(arg);
    return Init_CommunicationMessage_teams_rol(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_teams_tid
{
public:
  explicit Init_CommunicationMessage_teams_tid(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_teams_id teams_tid(::robot_communication_interfaces::msg::CommunicationMessage::_teams_tid_type arg)
  {
    msg_.teams_tid = std::move(arg);
    return Init_CommunicationMessage_teams_id(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_suitability_values
{
public:
  explicit Init_CommunicationMessage_suitability_values(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_teams_tid suitability_values(::robot_communication_interfaces::msg::CommunicationMessage::_suitability_values_type arg)
  {
    msg_.suitability_values = std::move(arg);
    return Init_CommunicationMessage_teams_tid(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_suitability_roles
{
public:
  explicit Init_CommunicationMessage_suitability_roles(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_suitability_values suitability_roles(::robot_communication_interfaces::msg::CommunicationMessage::_suitability_roles_type arg)
  {
    msg_.suitability_roles = std::move(arg);
    return Init_CommunicationMessage_suitability_values(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_suitability_robots
{
public:
  explicit Init_CommunicationMessage_suitability_robots(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_suitability_roles suitability_robots(::robot_communication_interfaces::msg::CommunicationMessage::_suitability_robots_type arg)
  {
    msg_.suitability_robots = std::move(arg);
    return Init_CommunicationMessage_suitability_roles(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_receiver_robot_id
{
public:
  explicit Init_CommunicationMessage_receiver_robot_id(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_suitability_robots receiver_robot_id(::robot_communication_interfaces::msg::CommunicationMessage::_receiver_robot_id_type arg)
  {
    msg_.receiver_robot_id = std::move(arg);
    return Init_CommunicationMessage_suitability_robots(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_position
{
public:
  explicit Init_CommunicationMessage_position(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_receiver_robot_id position(::robot_communication_interfaces::msg::CommunicationMessage::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_CommunicationMessage_receiver_robot_id(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_robot_id
{
public:
  explicit Init_CommunicationMessage_robot_id(::robot_communication_interfaces::msg::CommunicationMessage & msg)
  : msg_(msg)
  {}
  Init_CommunicationMessage_position robot_id(::robot_communication_interfaces::msg::CommunicationMessage::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_CommunicationMessage_position(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

class Init_CommunicationMessage_message_type
{
public:
  Init_CommunicationMessage_message_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CommunicationMessage_robot_id message_type(::robot_communication_interfaces::msg::CommunicationMessage::_message_type_type arg)
  {
    msg_.message_type = std::move(arg);
    return Init_CommunicationMessage_robot_id(msg_);
  }

private:
  ::robot_communication_interfaces::msg::CommunicationMessage msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_communication_interfaces::msg::CommunicationMessage>()
{
  return robot_communication_interfaces::msg::builder::Init_CommunicationMessage_message_type();
}

}  // namespace robot_communication_interfaces

#endif  // ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__BUILDER_HPP_
