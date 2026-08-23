// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from idaws_msgs:msg/ClusterArray.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__CLUSTER_ARRAY__FUNCTIONS_H_
#define IDAWS_MSGS__MSG__DETAIL__CLUSTER_ARRAY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "idaws_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "idaws_msgs/msg/detail/cluster_array__struct.h"

/// Initialize msg/ClusterArray message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * idaws_msgs__msg__ClusterArray
 * )) before or use
 * idaws_msgs__msg__ClusterArray__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__ClusterArray__init(idaws_msgs__msg__ClusterArray * msg);

/// Finalize msg/ClusterArray message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
void
idaws_msgs__msg__ClusterArray__fini(idaws_msgs__msg__ClusterArray * msg);

/// Create msg/ClusterArray message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * idaws_msgs__msg__ClusterArray__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
idaws_msgs__msg__ClusterArray *
idaws_msgs__msg__ClusterArray__create();

/// Destroy msg/ClusterArray message.
/**
 * It calls
 * idaws_msgs__msg__ClusterArray__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
void
idaws_msgs__msg__ClusterArray__destroy(idaws_msgs__msg__ClusterArray * msg);

/// Check for msg/ClusterArray message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__ClusterArray__are_equal(const idaws_msgs__msg__ClusterArray * lhs, const idaws_msgs__msg__ClusterArray * rhs);

/// Copy a msg/ClusterArray message.
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
idaws_msgs__msg__ClusterArray__copy(
  const idaws_msgs__msg__ClusterArray * input,
  idaws_msgs__msg__ClusterArray * output);

/// Initialize array of msg/ClusterArray messages.
/**
 * It allocates the memory for the number of elements and calls
 * idaws_msgs__msg__ClusterArray__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__ClusterArray__Sequence__init(idaws_msgs__msg__ClusterArray__Sequence * array, size_t size);

/// Finalize array of msg/ClusterArray messages.
/**
 * It calls
 * idaws_msgs__msg__ClusterArray__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
void
idaws_msgs__msg__ClusterArray__Sequence__fini(idaws_msgs__msg__ClusterArray__Sequence * array);

/// Create array of msg/ClusterArray messages.
/**
 * It allocates the memory for the array and calls
 * idaws_msgs__msg__ClusterArray__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
idaws_msgs__msg__ClusterArray__Sequence *
idaws_msgs__msg__ClusterArray__Sequence__create(size_t size);

/// Destroy array of msg/ClusterArray messages.
/**
 * It calls
 * idaws_msgs__msg__ClusterArray__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
void
idaws_msgs__msg__ClusterArray__Sequence__destroy(idaws_msgs__msg__ClusterArray__Sequence * array);

/// Check for msg/ClusterArray message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_idaws_msgs
bool
idaws_msgs__msg__ClusterArray__Sequence__are_equal(const idaws_msgs__msg__ClusterArray__Sequence * lhs, const idaws_msgs__msg__ClusterArray__Sequence * rhs);

/// Copy an array of msg/ClusterArray messages.
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
idaws_msgs__msg__ClusterArray__Sequence__copy(
  const idaws_msgs__msg__ClusterArray__Sequence * input,
  idaws_msgs__msg__ClusterArray__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // IDAWS_MSGS__MSG__DETAIL__CLUSTER_ARRAY__FUNCTIONS_H_
