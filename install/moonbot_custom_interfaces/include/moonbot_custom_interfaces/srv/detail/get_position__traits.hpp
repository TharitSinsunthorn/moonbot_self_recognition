// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from moonbot_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__TRAITS_HPP_
#define MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__TRAITS_HPP_

#include "moonbot_custom_interfaces/srv/detail/get_position__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<moonbot_custom_interfaces::srv::GetPosition_Request>()
{
  return "moonbot_custom_interfaces::srv::GetPosition_Request";
}

template<>
inline const char * name<moonbot_custom_interfaces::srv::GetPosition_Request>()
{
  return "moonbot_custom_interfaces/srv/GetPosition_Request";
}

template<>
struct has_fixed_size<moonbot_custom_interfaces::srv::GetPosition_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<moonbot_custom_interfaces::srv::GetPosition_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<moonbot_custom_interfaces::srv::GetPosition_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<moonbot_custom_interfaces::srv::GetPosition_Response>()
{
  return "moonbot_custom_interfaces::srv::GetPosition_Response";
}

template<>
inline const char * name<moonbot_custom_interfaces::srv::GetPosition_Response>()
{
  return "moonbot_custom_interfaces/srv/GetPosition_Response";
}

template<>
struct has_fixed_size<moonbot_custom_interfaces::srv::GetPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<moonbot_custom_interfaces::srv::GetPosition_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<moonbot_custom_interfaces::srv::GetPosition_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<moonbot_custom_interfaces::srv::GetPosition>()
{
  return "moonbot_custom_interfaces::srv::GetPosition";
}

template<>
inline const char * name<moonbot_custom_interfaces::srv::GetPosition>()
{
  return "moonbot_custom_interfaces/srv/GetPosition";
}

template<>
struct has_fixed_size<moonbot_custom_interfaces::srv::GetPosition>
  : std::integral_constant<
    bool,
    has_fixed_size<moonbot_custom_interfaces::srv::GetPosition_Request>::value &&
    has_fixed_size<moonbot_custom_interfaces::srv::GetPosition_Response>::value
  >
{
};

template<>
struct has_bounded_size<moonbot_custom_interfaces::srv::GetPosition>
  : std::integral_constant<
    bool,
    has_bounded_size<moonbot_custom_interfaces::srv::GetPosition_Request>::value &&
    has_bounded_size<moonbot_custom_interfaces::srv::GetPosition_Response>::value
  >
{
};

template<>
struct is_service<moonbot_custom_interfaces::srv::GetPosition>
  : std::true_type
{
};

template<>
struct is_service_request<moonbot_custom_interfaces::srv::GetPosition_Request>
  : std::true_type
{
};

template<>
struct is_service_response<moonbot_custom_interfaces::srv::GetPosition_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // MOONBOT_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__TRAITS_HPP_
