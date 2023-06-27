// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from moonbot_custom_interfaces:msg/JointAngles.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__STRUCT_H_
#define MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in msg/JointAngles in the package moonbot_custom_interfaces.
typedef struct moonbot_custom_interfaces__msg__JointAngles
{
  double joint1;
  double joint2;
  double joint3;
} moonbot_custom_interfaces__msg__JointAngles;

// Struct for a sequence of moonbot_custom_interfaces__msg__JointAngles.
typedef struct moonbot_custom_interfaces__msg__JointAngles__Sequence
{
  moonbot_custom_interfaces__msg__JointAngles * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} moonbot_custom_interfaces__msg__JointAngles__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__STRUCT_H_
