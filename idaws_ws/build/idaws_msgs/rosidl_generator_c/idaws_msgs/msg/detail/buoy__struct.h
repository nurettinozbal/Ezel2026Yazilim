// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from idaws_msgs:msg/Buoy.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY__STRUCT_H_
#define IDAWS_MSGS__MSG__DETAIL__BUOY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'label'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Buoy in the package idaws_msgs.
typedef struct idaws_msgs__msg__Buoy
{
  rosidl_runtime_c__String label;
  float confidence;
  int32_t x_min;
  int32_t y_min;
  int32_t x_max;
  int32_t y_max;
  float center_x;
  float center_y;
} idaws_msgs__msg__Buoy;

// Struct for a sequence of idaws_msgs__msg__Buoy.
typedef struct idaws_msgs__msg__Buoy__Sequence
{
  idaws_msgs__msg__Buoy * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} idaws_msgs__msg__Buoy__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY__STRUCT_H_
