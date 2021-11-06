// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robot_communication_interfaces:srv/CommunicationService.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__FUNCTIONS_H_
#define ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robot_communication_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "robot_communication_interfaces/srv/detail/communication_service__struct.h"

/// Initialize srv/CommunicationService message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_communication_interfaces__srv__CommunicationService_Request
 * )) before or use
 * robot_communication_interfaces__srv__CommunicationService_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
bool
robot_communication_interfaces__srv__CommunicationService_Request__init(robot_communication_interfaces__srv__CommunicationService_Request * msg);

/// Finalize srv/CommunicationService message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__srv__CommunicationService_Request__fini(robot_communication_interfaces__srv__CommunicationService_Request * msg);

/// Create srv/CommunicationService message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_communication_interfaces__srv__CommunicationService_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
robot_communication_interfaces__srv__CommunicationService_Request *
robot_communication_interfaces__srv__CommunicationService_Request__create();

/// Destroy srv/CommunicationService message.
/**
 * It calls
 * robot_communication_interfaces__srv__CommunicationService_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__srv__CommunicationService_Request__destroy(robot_communication_interfaces__srv__CommunicationService_Request * msg);


/// Initialize array of srv/CommunicationService messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_communication_interfaces__srv__CommunicationService_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
bool
robot_communication_interfaces__srv__CommunicationService_Request__Sequence__init(robot_communication_interfaces__srv__CommunicationService_Request__Sequence * array, size_t size);

/// Finalize array of srv/CommunicationService messages.
/**
 * It calls
 * robot_communication_interfaces__srv__CommunicationService_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__srv__CommunicationService_Request__Sequence__fini(robot_communication_interfaces__srv__CommunicationService_Request__Sequence * array);

/// Create array of srv/CommunicationService messages.
/**
 * It allocates the memory for the array and calls
 * robot_communication_interfaces__srv__CommunicationService_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
robot_communication_interfaces__srv__CommunicationService_Request__Sequence *
robot_communication_interfaces__srv__CommunicationService_Request__Sequence__create(size_t size);

/// Destroy array of srv/CommunicationService messages.
/**
 * It calls
 * robot_communication_interfaces__srv__CommunicationService_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__srv__CommunicationService_Request__Sequence__destroy(robot_communication_interfaces__srv__CommunicationService_Request__Sequence * array);

/// Initialize srv/CommunicationService message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_communication_interfaces__srv__CommunicationService_Response
 * )) before or use
 * robot_communication_interfaces__srv__CommunicationService_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
bool
robot_communication_interfaces__srv__CommunicationService_Response__init(robot_communication_interfaces__srv__CommunicationService_Response * msg);

/// Finalize srv/CommunicationService message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__srv__CommunicationService_Response__fini(robot_communication_interfaces__srv__CommunicationService_Response * msg);

/// Create srv/CommunicationService message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_communication_interfaces__srv__CommunicationService_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
robot_communication_interfaces__srv__CommunicationService_Response *
robot_communication_interfaces__srv__CommunicationService_Response__create();

/// Destroy srv/CommunicationService message.
/**
 * It calls
 * robot_communication_interfaces__srv__CommunicationService_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__srv__CommunicationService_Response__destroy(robot_communication_interfaces__srv__CommunicationService_Response * msg);


/// Initialize array of srv/CommunicationService messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_communication_interfaces__srv__CommunicationService_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
bool
robot_communication_interfaces__srv__CommunicationService_Response__Sequence__init(robot_communication_interfaces__srv__CommunicationService_Response__Sequence * array, size_t size);

/// Finalize array of srv/CommunicationService messages.
/**
 * It calls
 * robot_communication_interfaces__srv__CommunicationService_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__srv__CommunicationService_Response__Sequence__fini(robot_communication_interfaces__srv__CommunicationService_Response__Sequence * array);

/// Create array of srv/CommunicationService messages.
/**
 * It allocates the memory for the array and calls
 * robot_communication_interfaces__srv__CommunicationService_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
robot_communication_interfaces__srv__CommunicationService_Response__Sequence *
robot_communication_interfaces__srv__CommunicationService_Response__Sequence__create(size_t size);

/// Destroy array of srv/CommunicationService messages.
/**
 * It calls
 * robot_communication_interfaces__srv__CommunicationService_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__srv__CommunicationService_Response__Sequence__destroy(robot_communication_interfaces__srv__CommunicationService_Response__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_COMMUNICATION_INTERFACES__SRV__DETAIL__COMMUNICATION_SERVICE__FUNCTIONS_H_
