// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from moonbot_custom_interfaces:msg/JointAngles.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "moonbot_custom_interfaces/msg/detail/joint_angles__rosidl_typesupport_introspection_c.h"
#include "moonbot_custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "moonbot_custom_interfaces/msg/detail/joint_angles__functions.h"
#include "moonbot_custom_interfaces/msg/detail/joint_angles__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void JointAngles__rosidl_typesupport_introspection_c__JointAngles_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  moonbot_custom_interfaces__msg__JointAngles__init(message_memory);
}

void JointAngles__rosidl_typesupport_introspection_c__JointAngles_fini_function(void * message_memory)
{
  moonbot_custom_interfaces__msg__JointAngles__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember JointAngles__rosidl_typesupport_introspection_c__JointAngles_message_member_array[3] = {
  {
    "joint1",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moonbot_custom_interfaces__msg__JointAngles, joint1),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint2",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moonbot_custom_interfaces__msg__JointAngles, joint2),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "joint3",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moonbot_custom_interfaces__msg__JointAngles, joint3),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers JointAngles__rosidl_typesupport_introspection_c__JointAngles_message_members = {
  "moonbot_custom_interfaces__msg",  // message namespace
  "JointAngles",  // message name
  3,  // number of fields
  sizeof(moonbot_custom_interfaces__msg__JointAngles),
  JointAngles__rosidl_typesupport_introspection_c__JointAngles_message_member_array,  // message members
  JointAngles__rosidl_typesupport_introspection_c__JointAngles_init_function,  // function to initialize message memory (memory has to be allocated)
  JointAngles__rosidl_typesupport_introspection_c__JointAngles_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t JointAngles__rosidl_typesupport_introspection_c__JointAngles_message_type_support_handle = {
  0,
  &JointAngles__rosidl_typesupport_introspection_c__JointAngles_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_moonbot_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, moonbot_custom_interfaces, msg, JointAngles)() {
  if (!JointAngles__rosidl_typesupport_introspection_c__JointAngles_message_type_support_handle.typesupport_identifier) {
    JointAngles__rosidl_typesupport_introspection_c__JointAngles_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &JointAngles__rosidl_typesupport_introspection_c__JointAngles_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
