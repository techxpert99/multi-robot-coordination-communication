// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__STRUCT_HPP_
#define ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__robot_communication_interfaces__msg__CommunicationMessage __attribute__((deprecated))
#else
# define DEPRECATED__robot_communication_interfaces__msg__CommunicationMessage __declspec(deprecated)
#endif

namespace robot_communication_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct CommunicationMessage_
{
  using Type = CommunicationMessage_<ContainerAllocator>;

  explicit CommunicationMessage_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_type = "";
      this->robot_id = "";
      std::fill<typename std::array<double, 3>::iterator, double>(this->position.begin(), this->position.end(), 0.0);
      this->receiver_robot_id = "";
      this->exchange_number = 0ull;
    }
  }

  explicit CommunicationMessage_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : message_type(_alloc),
    robot_id(_alloc),
    position(_alloc),
    receiver_robot_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->message_type = "";
      this->robot_id = "";
      std::fill<typename std::array<double, 3>::iterator, double>(this->position.begin(), this->position.end(), 0.0);
      this->receiver_robot_id = "";
      this->exchange_number = 0ull;
    }
  }

  // field types and members
  using _message_type_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _message_type_type message_type;
  using _robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _robot_id_type robot_id;
  using _position_type =
    std::array<double, 3>;
  _position_type position;
  using _receiver_robot_id_type =
    std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>;
  _receiver_robot_id_type receiver_robot_id;
  using _suitability_robots_type =
    rosidl_runtime_cpp::BoundedVector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, 1024, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _suitability_robots_type suitability_robots;
  using _suitability_roles_type =
    rosidl_runtime_cpp::BoundedVector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, 1024, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _suitability_roles_type suitability_roles;
  using _suitability_values_type =
    rosidl_runtime_cpp::BoundedVector<float, 1024, typename ContainerAllocator::template rebind<float>::other>;
  _suitability_values_type suitability_values;
  using _teams_tid_type =
    rosidl_runtime_cpp::BoundedVector<uint32_t, 1024, typename ContainerAllocator::template rebind<uint32_t>::other>;
  _teams_tid_type teams_tid;
  using _teams_id_type =
    rosidl_runtime_cpp::BoundedVector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, 1024, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _teams_id_type teams_id;
  using _teams_rol_type =
    rosidl_runtime_cpp::BoundedVector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, 1024, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other>;
  _teams_rol_type teams_rol;
  using _teams_su_type =
    rosidl_runtime_cpp::BoundedVector<float, 1024, typename ContainerAllocator::template rebind<float>::other>;
  _teams_su_type teams_su;
  using _exchange_number_type =
    uint64_t;
  _exchange_number_type exchange_number;

  // setters for named parameter idiom
  Type & set__message_type(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->message_type = _arg;
    return *this;
  }
  Type & set__robot_id(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__position(
    const std::array<double, 3> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__receiver_robot_id(
    const std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other> & _arg)
  {
    this->receiver_robot_id = _arg;
    return *this;
  }
  Type & set__suitability_robots(
    const rosidl_runtime_cpp::BoundedVector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, 1024, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->suitability_robots = _arg;
    return *this;
  }
  Type & set__suitability_roles(
    const rosidl_runtime_cpp::BoundedVector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, 1024, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->suitability_roles = _arg;
    return *this;
  }
  Type & set__suitability_values(
    const rosidl_runtime_cpp::BoundedVector<float, 1024, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->suitability_values = _arg;
    return *this;
  }
  Type & set__teams_tid(
    const rosidl_runtime_cpp::BoundedVector<uint32_t, 1024, typename ContainerAllocator::template rebind<uint32_t>::other> & _arg)
  {
    this->teams_tid = _arg;
    return *this;
  }
  Type & set__teams_id(
    const rosidl_runtime_cpp::BoundedVector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, 1024, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->teams_id = _arg;
    return *this;
  }
  Type & set__teams_rol(
    const rosidl_runtime_cpp::BoundedVector<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>, 1024, typename ContainerAllocator::template rebind<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other>>::other> & _arg)
  {
    this->teams_rol = _arg;
    return *this;
  }
  Type & set__teams_su(
    const rosidl_runtime_cpp::BoundedVector<float, 1024, typename ContainerAllocator::template rebind<float>::other> & _arg)
  {
    this->teams_su = _arg;
    return *this;
  }
  Type & set__exchange_number(
    const uint64_t & _arg)
  {
    this->exchange_number = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_communication_interfaces__msg__CommunicationMessage
    std::shared_ptr<robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_communication_interfaces__msg__CommunicationMessage
    std::shared_ptr<robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CommunicationMessage_ & other) const
  {
    if (this->message_type != other.message_type) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    if (this->receiver_robot_id != other.receiver_robot_id) {
      return false;
    }
    if (this->suitability_robots != other.suitability_robots) {
      return false;
    }
    if (this->suitability_roles != other.suitability_roles) {
      return false;
    }
    if (this->suitability_values != other.suitability_values) {
      return false;
    }
    if (this->teams_tid != other.teams_tid) {
      return false;
    }
    if (this->teams_id != other.teams_id) {
      return false;
    }
    if (this->teams_rol != other.teams_rol) {
      return false;
    }
    if (this->teams_su != other.teams_su) {
      return false;
    }
    if (this->exchange_number != other.exchange_number) {
      return false;
    }
    return true;
  }
  bool operator!=(const CommunicationMessage_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CommunicationMessage_

// alias to use template instance with default allocator
using CommunicationMessage =
  robot_communication_interfaces::msg::CommunicationMessage_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_communication_interfaces

#endif  // ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__STRUCT_HPP_
