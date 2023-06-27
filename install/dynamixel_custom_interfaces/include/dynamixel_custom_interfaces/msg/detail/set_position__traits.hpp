// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from dynamixel_custom_interfaces:msg/SetPosition.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__TRAITS_HPP_
#define DYNAMIXEL_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__TRAITS_HPP_

#include "dynamixel_custom_interfaces/msg/detail/set_position__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<dynamixel_custom_interfaces::msg::SetPosition>()
{
  return "dynamixel_custom_interfaces::msg::SetPosition";
}

template<>
inline const char * name<dynamixel_custom_interfaces::msg::SetPosition>()
{
  return "dynamixel_custom_interfaces/msg/SetPosition";
}

template<>
struct has_fixed_size<dynamixel_custom_interfaces::msg::SetPosition>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<dynamixel_custom_interfaces::msg::SetPosition>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<dynamixel_custom_interfaces::msg::SetPosition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DYNAMIXEL_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__TRAITS_HPP_
