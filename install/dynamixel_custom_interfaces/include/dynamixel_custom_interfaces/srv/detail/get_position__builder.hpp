// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from dynamixel_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_
#define DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_

#include "dynamixel_custom_interfaces/srv/detail/get_position__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace dynamixel_custom_interfaces
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
  ::dynamixel_custom_interfaces::srv::GetPosition_Request id(::dynamixel_custom_interfaces::srv::GetPosition_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_custom_interfaces::srv::GetPosition_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_custom_interfaces::srv::GetPosition_Request>()
{
  return dynamixel_custom_interfaces::srv::builder::Init_GetPosition_Request_id();
}

}  // namespace dynamixel_custom_interfaces


namespace dynamixel_custom_interfaces
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
  ::dynamixel_custom_interfaces::srv::GetPosition_Response position(::dynamixel_custom_interfaces::srv::GetPosition_Response::_position_type arg)
  {
    msg_.position = std::move(arg);
    return std::move(msg_);
  }

private:
  ::dynamixel_custom_interfaces::srv::GetPosition_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::dynamixel_custom_interfaces::srv::GetPosition_Response>()
{
  return dynamixel_custom_interfaces::srv::builder::Init_GetPosition_Response_position();
}

}  // namespace dynamixel_custom_interfaces

#endif  // DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__BUILDER_HPP_
