// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from moonbot_custom_interfaces:msg/SetPosition.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__STRUCT_HPP_
#define MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__moonbot_custom_interfaces__msg__SetPosition __attribute__((deprecated))
#else
# define DEPRECATED__moonbot_custom_interfaces__msg__SetPosition __declspec(deprecated)
#endif

namespace moonbot_custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SetPosition_
{
  using Type = SetPosition_<ContainerAllocator>;

  explicit SetPosition_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->position = 0l;
    }
  }

  explicit SetPosition_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
      this->position = 0l;
    }
  }

  // field types and members
  using _id_type =
    uint8_t;
  _id_type id;
  using _position_type =
    int32_t;
  _position_type position;

  // setters for named parameter idiom
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__position(
    const int32_t & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator> *;
  using ConstRawPtr =
    const moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moonbot_custom_interfaces__msg__SetPosition
    std::shared_ptr<moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moonbot_custom_interfaces__msg__SetPosition
    std::shared_ptr<moonbot_custom_interfaces::msg::SetPosition_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SetPosition_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const SetPosition_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SetPosition_

// alias to use template instance with default allocator
using SetPosition =
  moonbot_custom_interfaces::msg::SetPosition_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace moonbot_custom_interfaces

#endif  // MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__SET_POSITION__STRUCT_HPP_
