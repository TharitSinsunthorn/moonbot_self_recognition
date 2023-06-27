// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from dynamixel_custom_interfaces:srv/GetPosition.idl
// generated code does not contain a copyright notice

#ifndef DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_HPP_
#define DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__dynamixel_custom_interfaces__srv__GetPosition_Request __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_custom_interfaces__srv__GetPosition_Request __declspec(deprecated)
#endif

namespace dynamixel_custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPosition_Request_
{
  using Type = GetPosition_Request_<ContainerAllocator>;

  explicit GetPosition_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
    }
  }

  explicit GetPosition_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0;
    }
  }

  // field types and members
  using _id_type =
    uint8_t;
  _id_type id;

  // setters for named parameter idiom
  Type & set__id(
    const uint8_t & _arg)
  {
    this->id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_custom_interfaces__srv__GetPosition_Request
    std::shared_ptr<dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_custom_interfaces__srv__GetPosition_Request
    std::shared_ptr<dynamixel_custom_interfaces::srv::GetPosition_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPosition_Request_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPosition_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPosition_Request_

// alias to use template instance with default allocator
using GetPosition_Request =
  dynamixel_custom_interfaces::srv::GetPosition_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dynamixel_custom_interfaces


#ifndef _WIN32
# define DEPRECATED__dynamixel_custom_interfaces__srv__GetPosition_Response __attribute__((deprecated))
#else
# define DEPRECATED__dynamixel_custom_interfaces__srv__GetPosition_Response __declspec(deprecated)
#endif

namespace dynamixel_custom_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetPosition_Response_
{
  using Type = GetPosition_Response_<ContainerAllocator>;

  explicit GetPosition_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position = 0l;
    }
  }

  explicit GetPosition_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->position = 0l;
    }
  }

  // field types and members
  using _position_type =
    int32_t;
  _position_type position;

  // setters for named parameter idiom
  Type & set__position(
    const int32_t & _arg)
  {
    this->position = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__dynamixel_custom_interfaces__srv__GetPosition_Response
    std::shared_ptr<dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__dynamixel_custom_interfaces__srv__GetPosition_Response
    std::shared_ptr<dynamixel_custom_interfaces::srv::GetPosition_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetPosition_Response_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetPosition_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetPosition_Response_

// alias to use template instance with default allocator
using GetPosition_Response =
  dynamixel_custom_interfaces::srv::GetPosition_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace dynamixel_custom_interfaces

namespace dynamixel_custom_interfaces
{

namespace srv
{

struct GetPosition
{
  using Request = dynamixel_custom_interfaces::srv::GetPosition_Request;
  using Response = dynamixel_custom_interfaces::srv::GetPosition_Response;
};

}  // namespace srv

}  // namespace dynamixel_custom_interfaces

#endif  // DYNAMIXEL_CUSTOM_INTERFACES__SRV__DETAIL__GET_POSITION__STRUCT_HPP_
