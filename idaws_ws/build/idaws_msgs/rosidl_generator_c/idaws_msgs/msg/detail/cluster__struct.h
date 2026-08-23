// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from idaws_msgs:msg/Cluster.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__CLUSTER__STRUCT_H_
#define IDAWS_MSGS__MSG__DETAIL__CLUSTER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Cluster in the package idaws_msgs.
/**
  * LiDAR taramasından ayrıştırılmış tek bir engel kümesi.
  * Koordinatlar araç gövde çerçevesinde: +x ileri, +y sol (metre).
 */
typedef struct idaws_msgs__msg__Cluster
{
  int32_t id;
  float center_x;
  float center_y;
  /// kümenin merkezine mesafe (m)
  float range;
  /// kümenin merkezinin kerteriz açısı (rad, +x'ten CCW)
  float bearing;
  /// kümedeki en yakın ışının mesafesi (m)
  float min_range;
  /// kümenin yaklaşık genişliği (m)
  float width;
  int32_t point_count;
} idaws_msgs__msg__Cluster;

// Struct for a sequence of idaws_msgs__msg__Cluster.
typedef struct idaws_msgs__msg__Cluster__Sequence
{
  idaws_msgs__msg__Cluster * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} idaws_msgs__msg__Cluster__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // IDAWS_MSGS__MSG__DETAIL__CLUSTER__STRUCT_H_
