// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from dynamixel_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_
#define DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/GetPosition in the package dynamixel_custom_interfaces.
typedef struct dynamixel_custom_interfaces__srv__GetPosition_Request
{
  uint8_t id;
} dynamixel_custom_interfaces__srv__GetPosition_Request;

// Struct for a sequence of dynamixel_custom_interfaces__srv__GetPosition_Request.
typedef struct dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence
{
  dynamixel_custom_interfaces__srv__GetPosition_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_custom_interfaces__srv__GetPosition_Request__Sequence;


// Constants defined in the message

// Struct defined in srv/GetPosition in the package dynamixel_custom_interfaces.
typedef struct dynamixel_custom_interfaces__srv__GetPosition_Response
{
  int32_t position;
} dynamixel_custom_interfaces__srv__GetPosition_Response;

// Struct for a sequence of dynamixel_custom_interfaces__srv__GetPosition_Response.
typedef struct dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence
{
  dynamixel_custom_interfaces__srv__GetPosition_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} dynamixel_custom_interfaces__srv__GetPosition_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_H_
