// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from moonbot_custom_interfaces:msg/SetPosition.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__TRAITS_HPP_
#define MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__TRAITS_HPP_

#include "moonbot_custom_interfaces/msg/detail/set_position__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<moonbot_custom_interfaces::msg::SetPosition>()
{
  return "moonbot_custom_interfaces::msg::SetPosition";
}

template<>
inline const char * name<moonbot_custom_interfaces::msg::SetPosition>()
{
  return "moonbot_custom_interfaces/msg/SetPosition";
}

template<>
struct has_fixed_size<moonbot_custom_interfaces::msg::SetPosition>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<moonbot_custom_interfaces::msg::SetPosition>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<moonbot_custom_interfaces::msg::SetPosition>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__TRAITS_HPP_
