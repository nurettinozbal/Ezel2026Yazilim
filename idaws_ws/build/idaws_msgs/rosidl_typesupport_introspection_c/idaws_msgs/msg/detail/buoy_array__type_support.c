// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from idaws_msgs:msg/BuoyArray.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "idaws_msgs/msg/detail/buoy_array__rosidl_typesupport_introspection_c.h"
#include "idaws_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "idaws_msgs/msg/detail/buoy_array__functions.h"
#include "idaws_msgs/msg/detail/buoy_array__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `buoys`
#include "idaws_msgs/msg/buoy.h"
// Member `buoys`
#include "idaws_msgs/msg/detail/buoy__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  idaws_msgs__msg__BuoyArray__init(message_memory);
}

void idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_fini_function(void * message_memory)
{
  idaws_msgs__msg__BuoyArray__fini(message_memory);
}

size_t idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__size_function__BuoyArray__buoys(
  const void * untyped_member)
{
  const idaws_msgs__msg__Buoy__Sequence * member =
    (const idaws_msgs__msg__Buoy__Sequence *)(untyped_member);
  return member->size;
}

const void * idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__get_const_function__BuoyArray__buoys(
  const void * untyped_member, size_t index)
{
  const idaws_msgs__msg__Buoy__Sequence * member =
    (const idaws_msgs__msg__Buoy__Sequence *)(untyped_member);
  return &member->data[index];
}

void * idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__get_function__BuoyArray__buoys(
  void * untyped_member, size_t index)
{
  idaws_msgs__msg__Buoy__Sequence * member =
    (idaws_msgs__msg__Buoy__Sequence *)(untyped_member);
  return &member->data[index];
}

void idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__fetch_function__BuoyArray__buoys(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const idaws_msgs__msg__Buoy * item =
    ((const idaws_msgs__msg__Buoy *)
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__get_const_function__BuoyArray__buoys(untyped_member, index));
  idaws_msgs__msg__Buoy * value =
    (idaws_msgs__msg__Buoy *)(untyped_value);
  *value = *item;
}

void idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__assign_function__BuoyArray__buoys(
  void * untyped_member, size_t index, const void * untyped_value)
{
  idaws_msgs__msg__Buoy * item =
    ((idaws_msgs__msg__Buoy *)
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__get_function__BuoyArray__buoys(untyped_member, index));
  const idaws_msgs__msg__Buoy * value =
    (const idaws_msgs__msg__Buoy *)(untyped_value);
  *item = *value;
}

bool idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__resize_function__BuoyArray__buoys(
  void * untyped_member, size_t size)
{
  idaws_msgs__msg__Buoy__Sequence * member =
    (idaws_msgs__msg__Buoy__Sequence *)(untyped_member);
  idaws_msgs__msg__Buoy__Sequence__fini(member);
  return idaws_msgs__msg__Buoy__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_member_array[4] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__BuoyArray, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "buoys",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__BuoyArray, buoys),  // bytes offset in struct
    NULL,  // default value
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__size_function__BuoyArray__buoys,  // size() function pointer
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__get_const_function__BuoyArray__buoys,  // get_const(index) function pointer
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__get_function__BuoyArray__buoys,  // get(index) function pointer
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__fetch_function__BuoyArray__buoys,  // fetch(index, &value) function pointer
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__assign_function__BuoyArray__buoys,  // assign(index, value) function pointer
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__resize_function__BuoyArray__buoys  // resize(index) function pointer
  },
  {
    "frame_width",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__BuoyArray, frame_width),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "frame_height",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(idaws_msgs__msg__BuoyArray, frame_height),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_members = {
  "idaws_msgs__msg",  // message namespace
  "BuoyArray",  // message name
  4,  // number of fields
  sizeof(idaws_msgs__msg__BuoyArray),
  idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_member_array,  // message members
  idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_init_function,  // function to initialize message memory (memory has to be allocated)
  idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_type_support_handle = {
  0,
  &idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_idaws_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, idaws_msgs, msg, BuoyArray)() {
  idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, idaws_msgs, msg, Buoy)();
  if (!idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_type_support_handle.typesupport_identifier) {
    idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &idaws_msgs__msg__BuoyArray__rosidl_typesupport_introspection_c__BuoyArray_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
