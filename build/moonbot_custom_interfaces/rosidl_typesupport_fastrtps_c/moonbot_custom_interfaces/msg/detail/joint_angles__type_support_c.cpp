// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from moonbot_custom_interfaces:msg/JointAngles.idl
// generated code does not contain a copyright notice
#include "moonbot_custom_interfaces/msg/detail/joint_angles__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "moonbot_custom_interfaces/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "moonbot_custom_interfaces/msg/detail/joint_angles__struct.h"
#include "moonbot_custom_interfaces/msg/detail/joint_angles__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _JointAngles__ros_msg_type = moonbot_custom_interfaces__msg__JointAngles;

static bool _JointAngles__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _JointAngles__ros_msg_type * ros_message = static_cast<const _JointAngles__ros_msg_type *>(untyped_ros_message);
  // Field name: joint1
  {
    cdr << ros_message->joint1;
  }

  // Field name: joint2
  {
    cdr << ros_message->joint2;
  }

  // Field name: joint3
  {
    cdr << ros_message->joint3;
  }

  return true;
}

static bool _JointAngles__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _JointAngles__ros_msg_type * ros_message = static_cast<_JointAngles__ros_msg_type *>(untyped_ros_message);
  // Field name: joint1
  {
    cdr >> ros_message->joint1;
  }

  // Field name: joint2
  {
    cdr >> ros_message->joint2;
  }

  // Field name: joint3
  {
    cdr >> ros_message->joint3;
  }

  return true;
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moonbot_custom_interfaces
size_t get_serialized_size_moonbot_custom_interfaces__msg__JointAngles(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _JointAngles__ros_msg_type * ros_message = static_cast<const _JointAngles__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name joint1
  {
    size_t item_size = sizeof(ros_message->joint1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint2
  {
    size_t item_size = sizeof(ros_message->joint2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint3
  {
    size_t item_size = sizeof(ros_message->joint3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _JointAngles__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_moonbot_custom_interfaces__msg__JointAngles(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_moonbot_custom_interfaces
size_t max_serialized_size_moonbot_custom_interfaces__msg__JointAngles(
  bool & full_bounded,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;
  (void)full_bounded;

  // member: joint1
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: joint2
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: joint3
  {
    size_t array_size = 1;

    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  return current_alignment - initial_alignment;
}

static size_t _JointAngles__max_serialized_size(bool & full_bounded)
{
  return max_serialized_size_moonbot_custom_interfaces__msg__JointAngles(
    full_bounded, 0);
}


static message_type_support_callbacks_t __callbacks_JointAngles = {
  "moonbot_custom_interfaces::msg",
  "JointAngles",
  _JointAngles__cdr_serialize,
  _JointAngles__cdr_deserialize,
  _JointAngles__get_serialized_size,
  _JointAngles__max_serialized_size
};

static rosidl_message_type_support_t _JointAngles__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_JointAngles,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, moonbot_custom_interfaces, msg, JointAngles)() {
  return &_JointAngles__type_support;
}

#if defined(__cplusplus)
}
#endif
