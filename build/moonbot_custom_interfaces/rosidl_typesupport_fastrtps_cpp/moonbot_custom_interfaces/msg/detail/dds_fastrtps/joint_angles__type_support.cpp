// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from moonbot_custom_interfaces:msg/JointAngles.idl
// generated code does not contain a copyright notice
#include "moonbot_custom_interfaces/msg/detail/joint_angles__rosidl_typesupport_fastrtps_cpp.hpp"
#include "moonbot_custom_interfaces/msg/detail/joint_angles__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace moonbot_custom_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_moonbot_custom_interfaces
cdr_serialize(
  const moonbot_custom_interfaces::msg::JointAngles & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: joint1
  cdr << ros_message.joint1;
  // Member: joint2
  cdr << ros_message.joint2;
  // Member: joint3
  cdr << ros_message.joint3;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_moonbot_custom_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  moonbot_custom_interfaces::msg::JointAngles & ros_message)
{
  // Member: joint1
  cdr >> ros_message.joint1;

  // Member: joint2
  cdr >> ros_message.joint2;

  // Member: joint3
  cdr >> ros_message.joint3;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_moonbot_custom_interfaces
get_serialized_size(
  const moonbot_custom_interfaces::msg::JointAngles & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: joint1
  {
    size_t item_size = sizeof(ros_message.joint1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint2
  {
    size_t item_size = sizeof(ros_message.joint2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint3
  {
    size_t item_size = sizeof(ros_message.joint3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_moonbot_custom_interfaces
max_serialized_size_JointAngles(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;


  // Member: joint1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: joint2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: joint3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static bool _JointAngles__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const moonbot_custom_interfaces::msg::JointAngles *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _JointAngles__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<moonbot_custom_interfaces::msg::JointAngles *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _JointAngles__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const moonbot_custom_interfaces::msg::JointAngles *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _JointAngles__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_JointAngles(full_bounded, 0);
}

static message_type_support_callbacks_t _JointAngles__callbacks = {
  "moonbot_custom_interfaces::msg",
  "JointAngles",
  _JointAngles__cdr_serialize,
  _JointAngles__cdr_deserialize,
  _JointAngles__get_serialized_size,
  _JointAngles__max_serialized_size
};

static rosidl_message_type_support_t _JointAngles__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_JointAngles__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace moonbot_custom_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_moonbot_custom_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<moonbot_custom_interfaces::msg::JointAngles>()
{
  return &moonbot_custom_interfaces::msg::typesupport_fastrtps_cpp::_JointAngles__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, moonbot_custom_interfaces, msg, JointAngles)() {
  return &moonbot_custom_interfaces::msg::typesupport_fastrtps_cpp::_JointAngles__handle;
}

#ifdef __cplusplus
}
#endif
