// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from idaws_msgs:msg/Buoy.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY__FUNCTIONS_H_
#define IDAWS_MSGS__MSG__DETAIL__BUOY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "idaws_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "idaws_msgs/msg/detail/buoy__struct.h"

/// Initialize msg/Buoy message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * idaws_msgs__msg__Buoy
 * )) before or use
 * idaws_msgs__msg__Buoy__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__Buoy__init(idaws_msgs__msg__Buoy * msg);

/// Finalize msg/Buoy message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
void
idaws_msgs__msg__Buoy__fini(idaws_msgs__msg__Buoy * msg);

/// Create msg/Buoy message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * idaws_msgs__msg__Buoy__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
idaws_msgs__msg__Buoy *
idaws_msgs__msg__Buoy__create();

/// Destroy msg/Buoy message.
/**
 * It calls
 * idaws_msgs__msg__Buoy__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
void
idaws_msgs__msg__Buoy__destroy(idaws_msgs__msg__Buoy * msg);

/// Check for msg/Buoy message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__Buoy__are_equal(const idaws_msgs__msg__Buoy * lhs, const idaws_msgs__msg__Buoy * rhs);

/// Copy a msg/Buoy message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__Buoy__copy(
  const idaws_msgs__msg__Buoy * input,
  idaws_msgs__msg__Buoy * output);

/// Initialize array of msg/Buoy messages.
/**
 * It allocates the memory for the number of elements and calls
 * idaws_msgs__msg__Buoy__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__Buoy__Sequence__init(idaws_msgs__msg__Buoy__Sequence * array, size_t size);

/// Finalize array of msg/Buoy messages.
/**
 * It calls
 * idaws_msgs__msg__Buoy__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
void
idaws_msgs__msg__Buoy__Sequence__fini(idaws_msgs__msg__Buoy__Sequence * array);

/// Create array of msg/Buoy messages.
/**
 * It allocates the memory for the array and calls
 * idaws_msgs__msg__Buoy__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
idaws_msgs__msg__Buoy__Sequence *
idaws_msgs__msg__Buoy__Sequence__create(size_t size);

/// Destroy array of msg/Buoy messages.
/**
 * It calls
 * idaws_msgs__msg__Buoy__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
void
idaws_msgs__msg__Buoy__Sequence__destroy(idaws_msgs__msg__Buoy__Sequence * array);

/// Check for msg/Buoy message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__Buoy__Sequence__are_equal(const idaws_msgs__msg__Buoy__Sequence * lhs, const idaws_msgs__msg__Buoy__Sequence * rhs);

/// Copy an array of msg/Buoy messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__Buoy__Sequence__copy(
  const idaws_msgs__msg__Buoy__Sequence * input,
  idaws_msgs__msg__Buoy__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY__FUNCTIONS_H_
