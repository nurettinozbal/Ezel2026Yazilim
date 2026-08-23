// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from idaws_msgs:msg/Cluster.idl
// generated code does not contain a copyright notice
#include "idaws_msgs/msg/detail/cluster__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
idaws_msgs__msg__Cluster__init(idaws_msgs__msg__Cluster * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // center_x
  // center_y
  // range
  // bearing
  // min_range
  // width
  // point_count
  return true;
}

void
idaws_msgs__msg__Cluster__fini(idaws_msgs__msg__Cluster * msg)
{
  if (!msg) {
    return;
  }
  // id
  // center_x
  // center_y
  // range
  // bearing
  // min_range
  // width
  // point_count
}

bool
idaws_msgs__msg__Cluster__are_equal(const idaws_msgs__msg__Cluster * lhs, const idaws_msgs__msg__Cluster * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
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
  // range
  if (lhs->range != rhs->range) {
    return false;
  }
  // bearing
  if (lhs->bearing != rhs->bearing) {
    return false;
  }
  // min_range
  if (lhs->min_range != rhs->min_range) {
    return false;
  }
  // width
  if (lhs->width != rhs->width) {
    return false;
  }
  // point_count
  if (lhs->point_count != rhs->point_count) {
    return false;
  }
  return true;
}

bool
idaws_msgs__msg__Cluster__copy(
  const idaws_msgs__msg__Cluster * input,
  idaws_msgs__msg__Cluster * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // center_x
  output->center_x = input->center_x;
  // center_y
  output->center_y = input->center_y;
  // range
  output->range = input->range;
  // bearing
  output->bearing = input->bearing;
  // min_range
  output->min_range = input->min_range;
  // width
  output->width = input->width;
  // point_count
  output->point_count = input->point_count;
  return true;
}

idaws_msgs__msg__Cluster *
idaws_msgs__msg__Cluster__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__Cluster * msg = (idaws_msgs__msg__Cluster *)allocator.allocate(sizeof(idaws_msgs__msg__Cluster), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(idaws_msgs__msg__Cluster));
  bool success = idaws_msgs__msg__Cluster__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
idaws_msgs__msg__Cluster__destroy(idaws_msgs__msg__Cluster * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    idaws_msgs__msg__Cluster__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
idaws_msgs__msg__Cluster__Sequence__init(idaws_msgs__msg__Cluster__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__Cluster * data = NULL;

  if (size) {
    data = (idaws_msgs__msg__Cluster *)allocator.zero_allocate(size, sizeof(idaws_msgs__msg__Cluster), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = idaws_msgs__msg__Cluster__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        idaws_msgs__msg__Cluster__fini(&data[i - 1]);
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
idaws_msgs__msg__Cluster__Sequence__fini(idaws_msgs__msg__Cluster__Sequence * array)
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
      idaws_msgs__msg__Cluster__fini(&array->data[i]);
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

idaws_msgs__msg__Cluster__Sequence *
idaws_msgs__msg__Cluster__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  idaws_msgs__msg__Cluster__Sequence * array = (idaws_msgs__msg__Cluster__Sequence *)allocator.allocate(sizeof(idaws_msgs__msg__Cluster__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = idaws_msgs__msg__Cluster__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
idaws_msgs__msg__Cluster__Sequence__destroy(idaws_msgs__msg__Cluster__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    idaws_msgs__msg__Cluster__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
idaws_msgs__msg__Cluster__Sequence__are_equal(const idaws_msgs__msg__Cluster__Sequence * lhs, const idaws_msgs__msg__Cluster__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!idaws_msgs__msg__Cluster__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
idaws_msgs__msg__Cluster__Sequence__copy(
  const idaws_msgs__msg__Cluster__Sequence * input,
  idaws_msgs__msg__Cluster__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(idaws_msgs__msg__Cluster);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    idaws_msgs__msg__Cluster * data =
      (idaws_msgs__msg__Cluster *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!idaws_msgs__msg__Cluster__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          idaws_msgs__msg__Cluster__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!idaws_msgs__msg__Cluster__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
