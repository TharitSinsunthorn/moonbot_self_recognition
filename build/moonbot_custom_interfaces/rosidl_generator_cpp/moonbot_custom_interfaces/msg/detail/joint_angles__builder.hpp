// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from moonbot_custom_interfaces:msg/JointAngles.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__BUILDER_HPP_
#define MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__BUILDER_HPP_

#include "moonbot_custom_interfaces/msg/detail/joint_angles__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace moonbot_custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_JointAngles_joint3
{
public:
  explicit Init_JointAngles_joint3(::moonbot_custom_interfaces::msg::JointAngles & msg)
  : msg_(msg)
  {}
  ::moonbot_custom_interfaces::msg::JointAngles joint3(::moonbot_custom_interfaces::msg::JointAngles::_joint3_type arg)
  {
    msg_.joint3 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moonbot_custom_interfaces::msg::JointAngles msg_;
};

class Init_JointAngles_joint2
{
public:
  explicit Init_JointAngles_joint2(::moonbot_custom_interfaces::msg::JointAngles & msg)
  : msg_(msg)
  {}
  Init_JointAngles_joint3 joint2(::moonbot_custom_interfaces::msg::JointAngles::_joint2_type arg)
  {
    msg_.joint2 = std::move(arg);
    return Init_JointAngles_joint3(msg_);
  }

private:
  ::moonbot_custom_interfaces::msg::JointAngles msg_;
};

class Init_JointAngles_joint1
{
public:
  Init_JointAngles_joint1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_JointAngles_joint2 joint1(::moonbot_custom_interfaces::msg::JointAngles::_joint1_type arg)
  {
    msg_.joint1 = std::move(arg);
    return Init_JointAngles_joint2(msg_);
  }

private:
  ::moonbot_custom_interfaces::msg::JointAngles msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::moonbot_custom_interfaces::msg::JointAngles>()
{
  return moonbot_custom_interfaces::msg::builder::Init_JointAngles_joint1();
}

}  // namespace moonbot_custom_interfaces

#endif  // MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__BUILDER_HPP_
