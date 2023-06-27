// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from moonbot_custom_interfaces:msg/JointAngles.idl
// generated code does not contain a copyright notice
#include "moonbot_custom_interfaces/msg/detail/joint_angles__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
moonbot_custom_interfaces__msg__JointAngles__init(moonbot_custom_interfaces__msg__JointAngles * msg)
{
  if (!msg) {
    return false;
  }
  // joint1
  // joint2
  // joint3
  return true;
}

void
moonbot_custom_interfaces__msg__JointAngles__fini(moonbot_custom_interfaces__msg__JointAngles * msg)
{
  if (!msg) {
    return;
  }
  // joint1
  // joint2
  // joint3
}

bool
moonbot_custom_interfaces__msg__JointAngles__are_equal(const moonbot_custom_interfaces__msg__JointAngles * lhs, const moonbot_custom_interfaces__msg__JointAngles * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // joint1
  if (lhs->joint1 != rhs->joint1) {
    return false;
  }
  // joint2
  if (lhs->joint2 != rhs->joint2) {
    return false;
  }
  // joint3
  if (lhs->joint3 != rhs->joint3) {
    return false;
  }
  return true;
}

bool
moonbot_custom_interfaces__msg__JointAngles__copy(
  const moonbot_custom_interfaces__msg__JointAngles * input,
  moonbot_custom_interfaces__msg__JointAngles * output)
{
  if (!input || !output) {
    return false;
  }
  // joint1
  output->joint1 = input->joint1;
  // joint2
  output->joint2 = input->joint2;
  // joint3
  output->joint3 = input->joint3;
  return true;
}

moonbot_custom_interfaces__msg__JointAngles *
moonbot_custom_interfaces__msg__JointAngles__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moonbot_custom_interfaces__msg__JointAngles * msg = (moonbot_custom_interfaces__msg__JointAngles *)allocator.allocate(sizeof(moonbot_custom_interfaces__msg__JointAngles), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(moonbot_custom_interfaces__msg__JointAngles));
  bool success = moonbot_custom_interfaces__msg__JointAngles__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
moonbot_custom_interfaces__msg__JointAngles__destroy(moonbot_custom_interfaces__msg__JointAngles * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    moonbot_custom_interfaces__msg__JointAngles__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
moonbot_custom_interfaces__msg__JointAngles__Sequence__init(moonbot_custom_interfaces__msg__JointAngles__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moonbot_custom_interfaces__msg__JointAngles * data = NULL;

  if (size) {
    data = (moonbot_custom_interfaces__msg__JointAngles *)allocator.zero_allocate(size, sizeof(moonbot_custom_interfaces__msg__JointAngles), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = moonbot_custom_interfaces__msg__JointAngles__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        moonbot_custom_interfaces__msg__JointAngles__fini(&data[i - 1]);
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
moonbot_custom_interfaces__msg__JointAngles__Sequence__fini(moonbot_custom_interfaces__msg__JointAngles__Sequence * array)
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
      moonbot_custom_interfaces__msg__JointAngles__fini(&array->data[i]);
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

moonbot_custom_interfaces__msg__JointAngles__Sequence *
moonbot_custom_interfaces__msg__JointAngles__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  moonbot_custom_interfaces__msg__JointAngles__Sequence * array = (moonbot_custom_interfaces__msg__JointAngles__Sequence *)allocator.allocate(sizeof(moonbot_custom_interfaces__msg__JointAngles__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = moonbot_custom_interfaces__msg__JointAngles__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
moonbot_custom_interfaces__msg__JointAngles__Sequence__destroy(moonbot_custom_interfaces__msg__JointAngles__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    moonbot_custom_interfaces__msg__JointAngles__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
moonbot_custom_interfaces__msg__JointAngles__Sequence__are_equal(const moonbot_custom_interfaces__msg__JointAngles__Sequence * lhs, const moonbot_custom_interfaces__msg__JointAngles__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!moonbot_custom_interfaces__msg__JointAngles__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
moonbot_custom_interfaces__msg__JointAngles__Sequence__copy(
  const moonbot_custom_interfaces__msg__JointAngles__Sequence * input,
  moonbot_custom_interfaces__msg__JointAngles__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(moonbot_custom_interfaces__msg__JointAngles);
    moonbot_custom_interfaces__msg__JointAngles * data =
      (moonbot_custom_interfaces__msg__JointAngles *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!moonbot_custom_interfaces__msg__JointAngles__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          moonbot_custom_interfaces__msg__JointAngles__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!moonbot_custom_interfaces__msg__JointAngles__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
