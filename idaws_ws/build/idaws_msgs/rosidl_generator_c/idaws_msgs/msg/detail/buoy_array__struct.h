// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from idaws_msgs:msg/BuoyArray.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__STRUCT_H_
#define IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__STRUCT_H_

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
// Member 'buoys'
#include "idaws_msgs/msg/detail/buoy__struct.h"

/// Struct defined in msg/BuoyArray in the package idaws_msgs.
typedef struct idaws_msgs__msg__BuoyArray
{
  std_msgs__msg__Header header;
  idaws_msgs__msg__Buoy__Sequence buoys;
  int32_t frame_width;
  int32_t frame_height;
} idaws_msgs__msg__BuoyArray;

// Struct for a sequence of idaws_msgs__msg__BuoyArray.
typedef struct idaws_msgs__msg__BuoyArray__Sequence
{
  idaws_msgs__msg__BuoyArray * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} idaws_msgs__msg__BuoyArray__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__STRUCT_H_
