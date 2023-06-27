// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from moonbot_custom_interfaces:msg/JointAngles.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "moonbot_custom_interfaces/msg/detail/joint_angles__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace moonbot_custom_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void JointAngles_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) moonbot_custom_interfaces::msg::JointAngles(_init);
}

void JointAngles_fini_function(void * message_memory)
{
  auto typed_message = static_cast<moonbot_custom_interfaces::msg::JointAngles *>(message_memory);
  typed_message->~JointAngles();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember JointAngles_message_member_array[3] = {
  {
    "joint1",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moonbot_custom_interfaces::msg::JointAngles, joint1),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint2",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moonbot_custom_interfaces::msg::JointAngles, joint2),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "joint3",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(moonbot_custom_interfaces::msg::JointAngles, joint3),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers JointAngles_message_members = {
  "moonbot_custom_interfaces::msg",  // message namespace
  "JointAngles",  // message name
  3,  // number of fields
  sizeof(moonbot_custom_interfaces::msg::JointAngles),
  JointAngles_message_member_array,  // message members
  JointAngles_init_function,  // function to initialize message memory (memory has to be allocated)
  JointAngles_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t JointAngles_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &JointAngles_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace moonbot_custom_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<moonbot_custom_interfaces::msg::JointAngles>()
{
  return &::moonbot_custom_interfaces::msg::rosidl_typesupport_introspection_cpp::JointAngles_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, moonbot_custom_interfaces, msg, JointAngles)() {
  return &::moonbot_custom_interfaces::msg::rosidl_typesupport_introspection_cpp::JointAngles_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
