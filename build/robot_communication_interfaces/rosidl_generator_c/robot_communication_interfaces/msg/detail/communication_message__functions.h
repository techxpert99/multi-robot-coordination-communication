// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robot_communication_interfaces:msg/CommunicationMessage.idl
// generated code does not contain a copyright notice

#ifndef ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__FUNCTIONS_H_
#define ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "robot_communication_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "robot_communication_interfaces/msg/detail/communication_message__struct.h"

/// Initialize msg/CommunicationMessage message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_communication_interfaces__msg__CommunicationMessage
 * )) before or use
 * robot_communication_interfaces__msg__CommunicationMessage__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
bool
robot_communication_interfaces__msg__CommunicationMessage__init(robot_communication_interfaces__msg__CommunicationMessage * msg);

/// Finalize msg/CommunicationMessage message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__msg__CommunicationMessage__fini(robot_communication_interfaces__msg__CommunicationMessage * msg);

/// Create msg/CommunicationMessage message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_communication_interfaces__msg__CommunicationMessage__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
robot_communication_interfaces__msg__CommunicationMessage *
robot_communication_interfaces__msg__CommunicationMessage__create();

/// Destroy msg/CommunicationMessage message.
/**
 * It calls
 * robot_communication_interfaces__msg__CommunicationMessage__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__msg__CommunicationMessage__destroy(robot_communication_interfaces__msg__CommunicationMessage * msg);


/// Initialize array of msg/CommunicationMessage messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_communication_interfaces__msg__CommunicationMessage__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
bool
robot_communication_interfaces__msg__CommunicationMessage__Sequence__init(robot_communication_interfaces__msg__CommunicationMessage__Sequence * array, size_t size);

/// Finalize array of msg/CommunicationMessage messages.
/**
 * It calls
 * robot_communication_interfaces__msg__CommunicationMessage__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__msg__CommunicationMessage__Sequence__fini(robot_communication_interfaces__msg__CommunicationMessage__Sequence * array);

/// Create array of msg/CommunicationMessage messages.
/**
 * It allocates the memory for the array and calls
 * robot_communication_interfaces__msg__CommunicationMessage__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
robot_communication_interfaces__msg__CommunicationMessage__Sequence *
robot_communication_interfaces__msg__CommunicationMessage__Sequence__create(size_t size);

/// Destroy array of msg/CommunicationMessage messages.
/**
 * It calls
 * robot_communication_interfaces__msg__CommunicationMessage__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_communication_interfaces
void
robot_communication_interfaces__msg__CommunicationMessage__Sequence__destroy(robot_communication_interfaces__msg__CommunicationMessage__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_COMMUNICATION_INTERFACES__MSG__DETAIL__COMMUNICATION_MESSAGE__FUNCTIONS_H_
