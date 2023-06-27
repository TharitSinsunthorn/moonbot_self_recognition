// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice
#include "dynamixel_custom_interfaces/srv/detail/get_position__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
dynamixel_custom_interfaces__srv__GetPosition_Request__init(dynamixel_custom_interfaces__srv__GetPosition_Request * msg)
{
  if (!msg) {
    return false;
  }
  // id
  return true;
}

void
dynamixel_custom_interfaces__srv__GetPosition_Request__fini(dynamixel_custom_interfaces__srv__GetPosition_Request * msg)
{
  if (!msg) {
    return;
  }
  // id
}

bool
dynamixel_custom_interfaces__srv__GetPosition_Request__are_equal(const dynamixel_custom_interfaces__srv__GetPosition_Request * lhs, const dynamixel_custom_interfaces__srv__GetPosition_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  return true;
}

bool
dynamixel_custom_interfaces__srv__GetPosition_Request__copy(
  const dynamixel_custom_interfaces__srv__GetPosition_Request * input,
  dynamixel_custom_interfaces__srv__GetPosition_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  return true;
}

dynamixel_custom_interfaces__srv__GetPosition_Request *
dynamixel_custom_interfaces__srv__GetPosition_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_custom_interfaces__srv__GetPosition_Request * msg = (dynamixel_custom_interfaces__srv__GetPosition_Request *)allocator.allocate(sizeof(dynamixel_custom_interfaces__srv__GetPosition_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_custom_interfaces__srv__GetPosition_Request));
  bool success = dynamixel_custom_interfaces__srv__GetPosition_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_custom_interfaces__srv__GetPosition_Request__destroy(dynamixel_custom_interfaces__srv__GetPosition_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_custom_interfaces__srv__GetPosition_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence__init(dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_custom_interfaces__srv__GetPosition_Request * data = NULL;

  if (size) {
    data = (dynamixel_custom_interfaces__srv__GetPosition_Request *)allocator.zero_allocate(size, sizeof(dynamixel_custom_interfaces__srv__GetPosition_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_custom_interfaces__srv__GetPosition_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_custom_interfaces__srv__GetPosition_Request__fini(&data[i - 1]);
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
dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence__fini(dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence * array)
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
      dynamixel_custom_interfaces__srv__GetPosition_Request__fini(&array->data[i]);
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

dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence *
dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence * array = (dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence *)allocator.allocate(sizeof(dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence__destroy(dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence__are_equal(const dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence * lhs, const dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_custom_interfaces__srv__GetPosition_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence__copy(
  const dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence * input,
  dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_custom_interfaces__srv__GetPosition_Request);
    dynamixel_custom_interfaces__srv__GetPosition_Request * data =
      (dynamixel_custom_interfaces__srv__GetPosition_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_custom_interfaces__srv__GetPosition_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dynamixel_custom_interfaces__srv__GetPosition_Request__fini(&data[i]);
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
    if (!dynamixel_custom_interfaces__srv__GetPosition_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
dynamixel_custom_interfaces__srv__GetPosition_Response__init(dynamixel_custom_interfaces__srv__GetPosition_Response * msg)
{
  if (!msg) {
    return false;
  }
  // position
  return true;
}

void
dynamixel_custom_interfaces__srv__GetPosition_Response__fini(dynamixel_custom_interfaces__srv__GetPosition_Response * msg)
{
  if (!msg) {
    return;
  }
  // position
}

bool
dynamixel_custom_interfaces__srv__GetPosition_Response__are_equal(const dynamixel_custom_interfaces__srv__GetPosition_Response * lhs, const dynamixel_custom_interfaces__srv__GetPosition_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // position
  if (lhs->position != rhs->position) {
    return false;
  }
  return true;
}

bool
dynamixel_custom_interfaces__srv__GetPosition_Response__copy(
  const dynamixel_custom_interfaces__srv__GetPosition_Response * input,
  dynamixel_custom_interfaces__srv__GetPosition_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // position
  output->position = input->position;
  return true;
}

dynamixel_custom_interfaces__srv__GetPosition_Response *
dynamixel_custom_interfaces__srv__GetPosition_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_custom_interfaces__srv__GetPosition_Response * msg = (dynamixel_custom_interfaces__srv__GetPosition_Response *)allocator.allocate(sizeof(dynamixel_custom_interfaces__srv__GetPosition_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_custom_interfaces__srv__GetPosition_Response));
  bool success = dynamixel_custom_interfaces__srv__GetPosition_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_custom_interfaces__srv__GetPosition_Response__destroy(dynamixel_custom_interfaces__srv__GetPosition_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_custom_interfaces__srv__GetPosition_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence__init(dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_custom_interfaces__srv__GetPosition_Response * data = NULL;

  if (size) {
    data = (dynamixel_custom_interfaces__srv__GetPosition_Response *)allocator.zero_allocate(size, sizeof(dynamixel_custom_interfaces__srv__GetPosition_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_custom_interfaces__srv__GetPosition_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_custom_interfaces__srv__GetPosition_Response__fini(&data[i - 1]);
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
dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence__fini(dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence * array)
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
      dynamixel_custom_interfaces__srv__GetPosition_Response__fini(&array->data[i]);
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

dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence *
dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence * array = (dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence *)allocator.allocate(sizeof(dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence__destroy(dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence__are_equal(const dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence * lhs, const dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_custom_interfaces__srv__GetPosition_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence__copy(
  const dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence * input,
  dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_custom_interfaces__srv__GetPosition_Response);
    dynamixel_custom_interfaces__srv__GetPosition_Response * data =
      (dynamixel_custom_interfaces__srv__GetPosition_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_custom_interfaces__srv__GetPosition_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          dynamixel_custom_interfaces__srv__GetPosition_Response__fini(&data[i]);
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
    if (!dynamixel_custom_interfaces__srv__GetPosition_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
