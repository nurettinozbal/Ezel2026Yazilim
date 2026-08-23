// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from idaws_msgs:msg/Cluster.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "idaws_msgs/msg/detail/cluster__rosidl_typesupport_introspection_c.h"
#include "idaws_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "idaws_msgs/msg/detail/cluster__functions.h"
#include "idaws_msgs/msg/detail/cluster__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  idaws_msgs__msg__Cluster__init(message_memory);
}

void idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_fini_function(void * message_memory)
{
  idaws_msgs__msg__Cluster__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_message_member_array[8] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__Cluster, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "center_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__Cluster, center_x),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "center_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__Cluster, center_y),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__Cluster, range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "bearing",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__Cluster, bearing),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "min_range",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__Cluster, min_range),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__Cluster, width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "point_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__Cluster, point_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_message_members = {
  "idaws_msgs__msg",  // message namespace
  "Cluster",  // message name
  8,  // number of fields
  sizeof(idaws_msgs__msg__Cluster),
  idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_message_member_array,  // message members
  idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_init_function,  // function to initialize message memory (memory has to be allocated)
  idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_message_type_support_handle = {
  0,
  &idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_idaws_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, idaws_msgs, msg, Cluster)() {
  if (!idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_message_type_support_handle.typesupport_identifier) {
    idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &idaws_msgs__msg__Cluster__rosidl_typesupport_introspection_c__Cluster_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
