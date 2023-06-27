// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from moonbot_custom_interfaces:msg/JointAngles.idl
// generated code does not contain a copyright notice

#ifndef MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__STRUCT_HPP_
#define MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__moonbot_custom_interfaces__msg__JointAngles __attribute__((deprecated))
#else
# define DEPRECATED__moonbot_custom_interfaces__msg__JointAngles __declspec(deprecated)
#endif

namespace moonbot_custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointAngles_
{
  using Type = JointAngles_<ContainerAllocator>;

  explicit JointAngles_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint1 = 0.0;
      this->joint2 = 0.0;
      this->joint3 = 0.0;
    }
  }

  explicit JointAngles_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->joint1 = 0.0;
      this->joint2 = 0.0;
      this->joint3 = 0.0;
    }
  }

  // field types and members
  using _joint1_type =
    double;
  _joint1_type joint1;
  using _joint2_type =
    double;
  _joint2_type joint2;
  using _joint3_type =
    double;
  _joint3_type joint3;

  // setters for named parameter idiom
  Type & set__joint1(
    const double & _arg)
  {
    this->joint1 = _arg;
    return *this;
  }
  Type & set__joint2(
    const double & _arg)
  {
    this->joint2 = _arg;
    return *this;
  }
  Type & set__joint3(
    const double & _arg)
  {
    this->joint3 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator> *;
  using ConstRawPtr =
    const moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__moonbot_custom_interfaces__msg__JointAngles
    std::shared_ptr<moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__moonbot_custom_interfaces__msg__JointAngles
    std::shared_ptr<moonbot_custom_interfaces::msg::JointAngles_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointAngles_ & other) const
  {
    if (this->joint1 != other.joint1) {
      return false;
    }
    if (this->joint2 != other.joint2) {
      return false;
    }
    if (this->joint3 != other.joint3) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointAngles_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointAngles_

// alias to use template instance with default allocator
using JointAngles =
  moonbot_custom_interfaces::msg::JointAngles_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace moonbot_custom_interfaces

#endif  // MOONBOT_CUSTOM_INTERFACES__MSG__DETAIL__JOINT_ANGLES__STRUCT_HPP_
