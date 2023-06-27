// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from moonbot_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_
#define MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_

#include "moonbot_custom_interfaces/srv/detail/get_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace moonbot_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetPosition_Request_id
{
public:
  Init_GetPosition_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::moonbot_custom_interfaces::srv::GetPosition_Request id(::moonbot_custom_interfaces::srv::GetPosition_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moonbot_custom_interfaces::srv::GetPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moonbot_custom_interfaces::srv::GetPosition_Request>()
{
  return moonbot_custom_interfaces::srv::builder::Init_GetPosition_Request_id();
}

}  // namespace moonbot_custom_interfaces


namespace moonbot_custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_GetPosition_Response_position
{
public:
  Init_GetPosition_Response_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::moonbot_custom_interfaces::srv::GetPosition_Response position(::moonbot_custom_interfaces::srv::GetPosition_Response::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::moonbot_custom_interfaces::srv::GetPosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::moonbot_custom_interfaces::srv::GetPosition_Response>()
{
  return moonbot_custom_interfaces::srv::builder::Init_GetPosition_Response_position();
}

}  // namespace moonbot_custom_interfaces

#endif  // MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_
