// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from idaws_msgs:msg/ClusterArray.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_H_
#define IDAWS_MSGS__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'clusters'
#include "idaws_msgs/msg/detail/cluster__struct.h"

/// Struct defined in msg/ClusterArray in the package idaws_msgs.
typedef struct idaws_msgs__msg__ClusterArray
{
  std_msgs__msg__Header header;
  idaws_msgs__msg__Cluster__Sequence clusters;
} idaws_msgs__msg__ClusterArray;

// Struct for a sequence of idaws_msgs__msg__ClusterArray.
typedef struct idaws_msgs__msg__ClusterArray__Sequence
{
  idaws_msgs__msg__ClusterArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} idaws_msgs__msg__ClusterArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IDAWS_MSGS__MSG__DETAIL__CLUSTER_ARRAY__STRUCT_H_
