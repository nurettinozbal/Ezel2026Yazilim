// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from idaws_msgs:msg/BuoyArray.idl
// generated code does not contain a copyright notice
#include "idaws_msgs/msg/detail/buoy_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `buoys`
#include "idaws_msgs/msg/detail/buoy__functions.h"

bool
idaws_msgs__msg__BuoyArray__init(idaws_msgs__msg__BuoyArray * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    idaws_msgs__msg__BuoyArray__fini(msg);
    return false;
  }
  // buoys
  if (!idaws_msgs__msg__Buoy__Sequence__init(&msg->buoys, 0)) {
    idaws_msgs__msg__BuoyArray__fini(msg);
    return false;
  }
  // frame_width
  // frame_height
  return true;
}

void
idaws_msgs__msg__BuoyArray__fini(idaws_msgs__msg__BuoyArray * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // buoys
  idaws_msgs__msg__Buoy__Sequence__fini(&msg->buoys);
  // frame_width
  // frame_height
}

bool
idaws_msgs__msg__BuoyArray__are_equal(const idaws_msgs__msg__BuoyArray * lhs, const idaws_msgs__msg__BuoyArray * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // buoys
  if (!idaws_msgs__msg__Buoy__Sequence__are_equal(
      &(lhs->buoys), &(rhs->buoys)))
  {
    return false;
  }
  // frame_width
  if (lhs->frame_width != rhs->frame_width) {
    return false;
  }
  // frame_height
  if (lhs->frame_height != rhs->frame_height) {
    return false;
  }
  return true;
}

bool
idaws_msgs__msg__BuoyArray__copy(
  const idaws_msgs__msg__BuoyArray * input,
  idaws_msgs__msg__BuoyArray * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // buoys
  if (!idaws_msgs__msg__Buoy__Sequence__copy(
      &(input->buoys), &(output->buoys)))
  {
    return false;
  }
  // frame_width
  output->frame_width = input->frame_width;
  // frame_height
  output->frame_height = input->frame_height;
  return true;
}

idaws_msgs__msg__BuoyArray *
idaws_msgs__msg__BuoyArray__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__BuoyArray * msg = (idaws_msgs__msg__BuoyArray *)allocator.allocate(sizeof(idaws_msgs__msg__BuoyArray), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(idaws_msgs__msg__BuoyArray));
  bool success = idaws_msgs__msg__BuoyArray__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
idaws_msgs__msg__BuoyArray__destroy(idaws_msgs__msg__BuoyArray * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    idaws_msgs__msg__BuoyArray__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
idaws_msgs__msg__BuoyArray__Sequence__init(idaws_msgs__msg__BuoyArray__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__BuoyArray * data = NULL;

  if (size) {
    data = (idaws_msgs__msg__BuoyArray *)allocator.zero_allocate(size, sizeof(idaws_msgs__msg__BuoyArray), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = idaws_msgs__msg__BuoyArray__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        idaws_msgs__msg__BuoyArray__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
idaws_msgs__msg__BuoyArray__Sequence__fini(idaws_msgs__msg__BuoyArray__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      idaws_msgs__msg__BuoyArray__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

idaws_msgs__msg__BuoyArray__Sequence *
idaws_msgs__msg__BuoyArray__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__BuoyArray__Sequence * array = (idaws_msgs__msg__BuoyArray__Sequence *)allocator.allocate(sizeof(idaws_msgs__msg__BuoyArray__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = idaws_msgs__msg__BuoyArray__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
idaws_msgs__msg__BuoyArray__Sequence__destroy(idaws_msgs__msg__BuoyArray__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    idaws_msgs__msg__BuoyArray__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
idaws_msgs__msg__BuoyArray__Sequence__are_equal(const idaws_msgs__msg__BuoyArray__Sequence * lhs, const idaws_msgs__msg__BuoyArray__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!idaws_msgs__msg__BuoyArray__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
idaws_msgs__msg__BuoyArray__Sequence__copy(
  const idaws_msgs__msg__BuoyArray__Sequence * input,
  idaws_msgs__msg__BuoyArray__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(idaws_msgs__msg__BuoyArray);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    idaws_msgs__msg__BuoyArray * data =
      (idaws_msgs__msg__BuoyArray *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!idaws_msgs__msg__BuoyArray__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          idaws_msgs__msg__BuoyArray__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!idaws_msgs__msg__BuoyArray__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
