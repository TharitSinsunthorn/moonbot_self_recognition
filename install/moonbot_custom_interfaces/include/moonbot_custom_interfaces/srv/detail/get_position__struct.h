// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from moonbot_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_
#define MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/GetPosition in the package moonbot_custom_interfaces.
typedef struct moonbot_custom_interfaces__srv__GetPosition_Request
{
  uint8_t id;
} moonbot_custom_interfaces__srv__GetPosition_Request;

// Struct for a sequence of moonbot_custom_interfaces__srv__GetPosition_Request.
typedef struct moonbot_custom_interfaces__srv__GetPosition_Request__Sequence
{
  moonbot_custom_interfaces__srv__GetPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moonbot_custom_interfaces__srv__GetPosition_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/GetPosition in the package moonbot_custom_interfaces.
typedef struct moonbot_custom_interfaces__srv__GetPosition_Response
{
  int32_t position;
} moonbot_custom_interfaces__srv__GetPosition_Response;

// Struct for a sequence of moonbot_custom_interfaces__srv__GetPosition_Response.
typedef struct moonbot_custom_interfaces__srv__GetPosition_Response__Sequence
{
  moonbot_custom_interfaces__srv__GetPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moonbot_custom_interfaces__srv__GetPosition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_
