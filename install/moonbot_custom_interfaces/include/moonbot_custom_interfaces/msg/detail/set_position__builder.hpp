// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from moonbot_custom_interfaces:msg/SetPosition.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__BUILDER_HPP_
#define MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__BUILDER_HPP_

#include "moonbot_custom_interfaces/msg/detail/set_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace moonbot_custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_SetPosition_position
{
public:
  explicit Init_SetPosition_position(::moonbot_custom_interfaces::msg::SetPosition & msg)
  : msg_(msg)
  {}
  ::moonbot_custom_interfaces::msg::SetPosition position(::moonbot_custom_interfaces::msg::SetPosition::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moonbot_custom_interfaces::msg::SetPosition msg_;
};

class Init_SetPosition_id
{
public:
  Init_SetPosition_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SetPosition_position id(::moonbot_custom_interfaces::msg::SetPosition::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_SetPosition_position(msg_);
  }

private:
  ::moonbot_custom_interfaces::msg::SetPosition msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::moonbot_custom_interfaces::msg::SetPosition>()
{
  return moonbot_custom_interfaces::msg::builder::Init_SetPosition_id();
}

}  // namespace moonbot_custom_interfaces

#endif  // MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__BUILDER_HPP_
