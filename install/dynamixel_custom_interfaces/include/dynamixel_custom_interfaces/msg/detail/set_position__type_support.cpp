// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from dynamixel_custom_interfaces:msg/SetPosition.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "dynamixel_custom_interfaces/msg/detail/set_position__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace dynamixel_custom_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void SetPosition_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) dynamixel_custom_interfaces::msg::SetPosition(_init);
}

void SetPosition_fini_function(void * message_memory)
{
  auto typed_message = static_cast<dynamixel_custom_interfaces::msg::SetPosition *>(message_memory);
  typed_message->~SetPosition();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember SetPosition_message_member_array[2] = {
  {
    "id",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_custom_interfaces::msg::SetPosition, id),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "position",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(dynamixel_custom_interfaces::msg::SetPosition, position),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers SetPosition_message_members = {
  "dynamixel_custom_interfaces::msg",  // message namespace
  "SetPosition",  // message name
  2,  // number of fields
  sizeof(dynamixel_custom_interfaces::msg::SetPosition),
  SetPosition_message_member_array,  // message members
  SetPosition_init_function,  // function to initialize message memory (memory has to be allocated)
  SetPosition_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t SetPosition_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &SetPosition_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace dynamixel_custom_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<dynamixel_custom_interfaces::msg::SetPosition>()
{
  return &::dynamixel_custom_interfaces::msg::rosidl_typesupport_introspection_cpp::SetPosition_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, dynamixel_custom_interfaces, msg, SetPosition)() {
  return &::dynamixel_custom_interfaces::msg::rosidl_typesupport_introspection_cpp::SetPosition_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
