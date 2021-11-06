// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__STRUCT_HPP_
#define ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'input_message'
#include "robot_communication_interfaces/msg/detail/communication_message__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_communication_interfaces__srv__CommunicationService_Request __attribute__((deprecated))
#else
# define DEPRECATED__robot_communication_interfaces__srv__CommunicationService_Request __declspec(deprecated)
#endif

namespace robot_communication_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CommunicationService_Request_
{
  using Type = CommunicationService_Request_<ContainerAllocator>;

  explicit CommunicationService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : input_message(_init)
  {
    (void)_init;
  }

  explicit CommunicationService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : input_message(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _input_message_type =
    robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator>;
  _input_message_type input_message;

  // setters for named parameter idiom
  Type & set__input_message(
    const robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator> & _arg)
  {
    this->input_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_communication_interfaces__srv__CommunicationService_Request
    std::shared_ptr<robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_communication_interfaces__srv__CommunicationService_Request
    std::shared_ptr<robot_communication_interfaces::srv::CommunicationService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CommunicationService_Request_ & other) const
  {
    if (this->input_message != other.input_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const CommunicationService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CommunicationService_Request_

// alias to use template instance with default allocator
using CommunicationService_Request =
  robot_communication_interfaces::srv::CommunicationService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_communication_interfaces


// Include directives for member types
// Member 'output_message'
// already included above
// #include "robot_communication_interfaces/msg/detail/communication_message__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__robot_communication_interfaces__srv__CommunicationService_Response __attribute__((deprecated))
#else
# define DEPRECATED__robot_communication_interfaces__srv__CommunicationService_Response __declspec(deprecated)
#endif

namespace robot_communication_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct CommunicationService_Response_
{
  using Type = CommunicationService_Response_<ContainerAllocator>;

  explicit CommunicationService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : output_message(_init)
  {
    (void)_init;
  }

  explicit CommunicationService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : output_message(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _output_message_type =
    robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator>;
  _output_message_type output_message;

  // setters for named parameter idiom
  Type & set__output_message(
    const robot_communication_interfaces::msg::CommunicationMessage_<ContainerAllocator> & _arg)
  {
    this->output_message = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_communication_interfaces__srv__CommunicationService_Response
    std::shared_ptr<robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_communication_interfaces__srv__CommunicationService_Response
    std::shared_ptr<robot_communication_interfaces::srv::CommunicationService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const CommunicationService_Response_ & other) const
  {
    if (this->output_message != other.output_message) {
      return false;
    }
    return true;
  }
  bool operator!=(const CommunicationService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct CommunicationService_Response_

// alias to use template instance with default allocator
using CommunicationService_Response =
  robot_communication_interfaces::srv::CommunicationService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace robot_communication_interfaces

namespace robot_communication_interfaces
{

namespace srv
{

struct CommunicationService
{
  using Request = robot_communication_interfaces::srv::CommunicationService_Request;
  using Response = robot_communication_interfaces::srv::CommunicationService_Response;
};

}  // namespace srv

}  // namespace robot_communication_interfaces

#endif  // ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__STRUCT_HPP_
