// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from dynamixel_custom_interfaces:msg/SetPosition.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "dynamixel_custom_interfaces/msg/detail/set_position__rosidl_typesupport_introspection_c.h"
#include "dynamixel_custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "dynamixel_custom_interfaces/msg/detail/set_position__functions.h"
#include "dynamixel_custom_interfaces/msg/detail/set_position__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void SetPosition__rosidl_typesupport_introspection_c__SetPosition_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  dynamixel_custom_interfaces__msg__SetPosition__init(message_memory);
}

void SetPosition__rosidl_typesupport_introspection_c__SetPosition_fini_function(void * message_memory)
{
  dynamixel_custom_interfaces__msg__SetPosition__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember SetPosition__rosidl_typesupport_introspection_c__SetPosition_message_member_array[2] = {
  {
    "id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_custom_interfaces__msg__SetPosition, id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_custom_interfaces__msg__SetPosition, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers SetPosition__rosidl_typesupport_introspection_c__SetPosition_message_members = {
  "dynamixel_custom_interfaces__msg",  // message namespace
  "SetPosition",  // message name
  2,  // number of fields
  sizeof(dynamixel_custom_interfaces__msg__SetPosition),
  SetPosition__rosidl_typesupport_introspection_c__SetPosition_message_member_array,  // message members
  SetPosition__rosidl_typesupport_introspection_c__SetPosition_init_function,  // function to initialize message memory (memory has to be allocated)
  SetPosition__rosidl_typesupport_introspection_c__SetPosition_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t SetPosition__rosidl_typesupport_introspection_c__SetPosition_message_type_support_handle = {
  0,
  &SetPosition__rosidl_typesupport_introspection_c__SetPosition_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_dynamixel_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, dynamixel_custom_interfaces, msg, SetPosition)() {
  if (!SetPosition__rosidl_typesupport_introspection_c__SetPosition_message_type_support_handle.typesupport_identifier) {
    SetPosition__rosidl_typesupport_introspection_c__SetPosition_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &SetPosition__rosidl_typesupport_introspection_c__SetPosition_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
