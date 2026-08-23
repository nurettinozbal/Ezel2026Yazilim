// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from idaws_msgs:msg/Buoy.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY__STRUCT_HPP_
#define IDAWS_MSGS__MSG__DETAIL__BUOY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__idaws_msgs__msg__Buoy __attribute__((deprecated))
#else
# define DEPRECATED__idaws_msgs__msg__Buoy __declspec(deprecated)
#endif

namespace idaws_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Buoy_
{
  using Type = Buoy_<ContainerAllocator>;

  explicit Buoy_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->confidence = 0.0f;
      this->x_min = 0l;
      this->y_min = 0l;
      this->x_max = 0l;
      this->y_max = 0l;
      this->center_x = 0.0f;
      this->center_y = 0.0f;
    }
  }

  explicit Buoy_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : label(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->label = "";
      this->confidence = 0.0f;
      this->x_min = 0l;
      this->y_min = 0l;
      this->x_max = 0l;
      this->y_max = 0l;
      this->center_x = 0.0f;
      this->center_y = 0.0f;
    }
  }

  // field types and members
  using _label_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _label_type label;
  using _confidence_type =
    float;
  _confidence_type confidence;
  using _x_min_type =
    int32_t;
  _x_min_type x_min;
  using _y_min_type =
    int32_t;
  _y_min_type y_min;
  using _x_max_type =
    int32_t;
  _x_max_type x_max;
  using _y_max_type =
    int32_t;
  _y_max_type y_max;
  using _center_x_type =
    float;
  _center_x_type center_x;
  using _center_y_type =
    float;
  _center_y_type center_y;

  // setters for named parameter idiom
  Type & set__label(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->label = _arg;
    return *this;
  }
  Type & set__confidence(
    const float & _arg)
  {
    this->confidence = _arg;
    return *this;
  }
  Type & set__x_min(
    const int32_t & _arg)
  {
    this->x_min = _arg;
    return *this;
  }
  Type & set__y_min(
    const int32_t & _arg)
  {
    this->y_min = _arg;
    return *this;
  }
  Type & set__x_max(
    const int32_t & _arg)
  {
    this->x_max = _arg;
    return *this;
  }
  Type & set__y_max(
    const int32_t & _arg)
  {
    this->y_max = _arg;
    return *this;
  }
  Type & set__center_x(
    const float & _arg)
  {
    this->center_x = _arg;
    return *this;
  }
  Type & set__center_y(
    const float & _arg)
  {
    this->center_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    idaws_msgs::msg::Buoy_<ContainerAllocator> *;
  using ConstRawPtr =
    const idaws_msgs::msg::Buoy_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<idaws_msgs::msg::Buoy_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<idaws_msgs::msg::Buoy_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      idaws_msgs::msg::Buoy_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<idaws_msgs::msg::Buoy_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      idaws_msgs::msg::Buoy_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<idaws_msgs::msg::Buoy_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<idaws_msgs::msg::Buoy_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<idaws_msgs::msg::Buoy_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__idaws_msgs__msg__Buoy
    std::shared_ptr<idaws_msgs::msg::Buoy_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__idaws_msgs__msg__Buoy
    std::shared_ptr<idaws_msgs::msg::Buoy_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Buoy_ & other) const
  {
    if (this->label != other.label) {
      return false;
    }
    if (this->confidence != other.confidence) {
      return false;
    }
    if (this->x_min != other.x_min) {
      return false;
    }
    if (this->y_min != other.y_min) {
      return false;
    }
    if (this->x_max != other.x_max) {
      return false;
    }
    if (this->y_max != other.y_max) {
      return false;
    }
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->center_y != other.center_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Buoy_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Buoy_

// alias to use template instance with default allocator
using Buoy =
  idaws_msgs::msg::Buoy_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace idaws_msgs

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY__STRUCT_HPP_
