// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_communication_interfaces/srv/detail/communication_service__rosidl_typesupport_introspection_c.h"
#include "robot_communication_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_communication_interfaces/srv/detail/communication_service__functions.h"
#include "robot_communication_interfaces/srv/detail/communication_service__struct.h"


// Include directives for member types
// Member `input_message`
#include "robot_communication_interfaces/msg/communication_message.h"
// Member `input_message`
#include "robot_communication_interfaces/msg/detail/communication_message__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_communication_interfaces__srv__CommunicationService_Request__init(message_memory);
}

void CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_fini_function(void * message_memory)
{
  robot_communication_interfaces__srv__CommunicationService_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_member_array[1] = {
  {
    "input_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces__srv__CommunicationService_Request, input_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_members = {
  "robot_communication_interfaces__srv",  // message namespace
  "CommunicationService_Request",  // message name
  1,  // number of fields
  sizeof(robot_communication_interfaces__srv__CommunicationService_Request),
  CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_member_array,  // message members
  CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_type_support_handle = {
  0,
  &CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_communication_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, srv, CommunicationService_Request)() {
  CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, msg, CommunicationMessage)();
  if (!CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_type_support_handle.typesupport_identifier) {
    CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CommunicationService_Request__rosidl_typesupport_introspection_c__CommunicationService_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "robot_communication_interfaces/srv/detail/communication_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "robot_communication_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "robot_communication_interfaces/srv/detail/communication_service__functions.h"
// already included above
// #include "robot_communication_interfaces/srv/detail/communication_service__struct.h"


// Include directives for member types
// Member `output_message`
// already included above
// #include "robot_communication_interfaces/msg/communication_message.h"
// Member `output_message`
// already included above
// #include "robot_communication_interfaces/msg/detail/communication_message__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_communication_interfaces__srv__CommunicationService_Response__init(message_memory);
}

void CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_fini_function(void * message_memory)
{
  robot_communication_interfaces__srv__CommunicationService_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_member_array[1] = {
  {
    "output_message",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_communication_interfaces__srv__CommunicationService_Response, output_message),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_members = {
  "robot_communication_interfaces__srv",  // message namespace
  "CommunicationService_Response",  // message name
  1,  // number of fields
  sizeof(robot_communication_interfaces__srv__CommunicationService_Response),
  CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_member_array,  // message members
  CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_type_support_handle = {
  0,
  &CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_communication_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, srv, CommunicationService_Response)() {
  CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, msg, CommunicationMessage)();
  if (!CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_type_support_handle.typesupport_identifier) {
    CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &CommunicationService_Response__rosidl_typesupport_introspection_c__CommunicationService_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "robot_communication_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "robot_communication_interfaces/srv/detail/communication_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_service_members = {
  "robot_communication_interfaces__srv",  // service namespace
  "CommunicationService",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_Request_message_type_support_handle,
  NULL  // response message
  // robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_Response_message_type_support_handle
};

static rosidl_service_type_support_t robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_service_type_support_handle = {
  0,
  &robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, srv, CommunicationService_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, srv, CommunicationService_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_communication_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, srv, CommunicationService)() {
  if (!robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_service_type_support_handle.typesupport_identifier) {
    robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, srv, CommunicationService_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_communication_interfaces, srv, CommunicationService_Response)()->data;
  }

  return &robot_communication_interfaces__srv__detail__communication_service__rosidl_typesupport_introspection_c__CommunicationService_service_type_support_handle;
}
