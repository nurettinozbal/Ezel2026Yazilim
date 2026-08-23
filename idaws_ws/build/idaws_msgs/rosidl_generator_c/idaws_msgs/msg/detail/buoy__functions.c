// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from idaws_msgs:msg/Buoy.idl
// generated code does not contain a copyright notice
#include "idaws_msgs/msg/detail/buoy__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `label`
#include "rosidl_runtime_c/string_functions.h"

bool
idaws_msgs__msg__Buoy__init(idaws_msgs__msg__Buoy * msg)
{
  if (!msg) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__init(&msg->label)) {
    idaws_msgs__msg__Buoy__fini(msg);
    return false;
  }
  // confidence
  // x_min
  // y_min
  // x_max
  // y_max
  // center_x
  // center_y
  return true;
}

void
idaws_msgs__msg__Buoy__fini(idaws_msgs__msg__Buoy * msg)
{
  if (!msg) {
    return;
  }
  // label
  rosidl_runtime_c__String__fini(&msg->label);
  // confidence
  // x_min
  // y_min
  // x_max
  // y_max
  // center_x
  // center_y
}

bool
idaws_msgs__msg__Buoy__are_equal(const idaws_msgs__msg__Buoy * lhs, const idaws_msgs__msg__Buoy * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->label), &(rhs->label)))
  {
    return false;
  }
  // confidence
  if (lhs->confidence != rhs->confidence) {
    return false;
  }
  // x_min
  if (lhs->x_min != rhs->x_min) {
    return false;
  }
  // y_min
  if (lhs->y_min != rhs->y_min) {
    return false;
  }
  // x_max
  if (lhs->x_max != rhs->x_max) {
    return false;
  }
  // y_max
  if (lhs->y_max != rhs->y_max) {
    return false;
  }
  // center_x
  if (lhs->center_x != rhs->center_x) {
    return false;
  }
  // center_y
  if (lhs->center_y != rhs->center_y) {
    return false;
  }
  return true;
}

bool
idaws_msgs__msg__Buoy__copy(
  const idaws_msgs__msg__Buoy * input,
  idaws_msgs__msg__Buoy * output)
{
  if (!input || !output) {
    return false;
  }
  // label
  if (!rosidl_runtime_c__String__copy(
      &(input->label), &(output->label)))
  {
    return false;
  }
  // confidence
  output->confidence = input->confidence;
  // x_min
  output->x_min = input->x_min;
  // y_min
  output->y_min = input->y_min;
  // x_max
  output->x_max = input->x_max;
  // y_max
  output->y_max = input->y_max;
  // center_x
  output->center_x = input->center_x;
  // center_y
  output->center_y = input->center_y;
  return true;
}

idaws_msgs__msg__Buoy *
idaws_msgs__msg__Buoy__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__Buoy * msg = (idaws_msgs__msg__Buoy *)allocator.allocate(sizeof(idaws_msgs__msg__Buoy), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(idaws_msgs__msg__Buoy));
  bool success = idaws_msgs__msg__Buoy__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
idaws_msgs__msg__Buoy__destroy(idaws_msgs__msg__Buoy * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    idaws_msgs__msg__Buoy__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
idaws_msgs__msg__Buoy__Sequence__init(idaws_msgs__msg__Buoy__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__Buoy * data = NULL;

  if (size) {
    data = (idaws_msgs__msg__Buoy *)allocator.zero_allocate(size, sizeof(idaws_msgs__msg__Buoy), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = idaws_msgs__msg__Buoy__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        idaws_msgs__msg__Buoy__fini(&data[i - 1]);
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
idaws_msgs__msg__Buoy__Sequence__fini(idaws_msgs__msg__Buoy__Sequence * array)
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
      idaws_msgs__msg__Buoy__fini(&array->data[i]);
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

idaws_msgs__msg__Buoy__Sequence *
idaws_msgs__msg__Buoy__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__Buoy__Sequence * array = (idaws_msgs__msg__Buoy__Sequence *)allocator.allocate(sizeof(idaws_msgs__msg__Buoy__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = idaws_msgs__msg__Buoy__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
idaws_msgs__msg__Buoy__Sequence__destroy(idaws_msgs__msg__Buoy__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    idaws_msgs__msg__Buoy__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
idaws_msgs__msg__Buoy__Sequence__are_equal(const idaws_msgs__msg__Buoy__Sequence * lhs, const idaws_msgs__msg__Buoy__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!idaws_msgs__msg__Buoy__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
idaws_msgs__msg__Buoy__Sequence__copy(
  const idaws_msgs__msg__Buoy__Sequence * input,
  idaws_msgs__msg__Buoy__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(idaws_msgs__msg__Buoy);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    idaws_msgs__msg__Buoy * data =
      (idaws_msgs__msg__Buoy *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!idaws_msgs__msg__Buoy__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          idaws_msgs__msg__Buoy__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!idaws_msgs__msg__Buoy__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
