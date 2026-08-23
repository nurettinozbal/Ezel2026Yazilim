// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from idaws_msgs:msg/Cluster.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__CLUSTER__STRUCT_HPP_
#define IDAWS_MSGS__MSG__DETAIL__CLUSTER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__idaws_msgs__msg__Cluster __attribute__((deprecated))
#else
# define DEPRECATED__idaws_msgs__msg__Cluster __declspec(deprecated)
#endif

namespace idaws_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Cluster_
{
  using Type = Cluster_<ContainerAllocator>;

  explicit Cluster_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->center_x = 0.0f;
      this->center_y = 0.0f;
      this->range = 0.0f;
      this->bearing = 0.0f;
      this->min_range = 0.0f;
      this->width = 0.0f;
      this->point_count = 0l;
    }
  }

  explicit Cluster_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0l;
      this->center_x = 0.0f;
      this->center_y = 0.0f;
      this->range = 0.0f;
      this->bearing = 0.0f;
      this->min_range = 0.0f;
      this->width = 0.0f;
      this->point_count = 0l;
    }
  }

  // field types and members
  using _id_type =
    int32_t;
  _id_type id;
  using _center_x_type =
    float;
  _center_x_type center_x;
  using _center_y_type =
    float;
  _center_y_type center_y;
  using _range_type =
    float;
  _range_type range;
  using _bearing_type =
    float;
  _bearing_type bearing;
  using _min_range_type =
    float;
  _min_range_type min_range;
  using _width_type =
    float;
  _width_type width;
  using _point_count_type =
    int32_t;
  _point_count_type point_count;

  // setters for named parameter idiom
  Type & set__id(
    const int32_t & _arg)
  {
    this->id = _arg;
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
  Type & set__range(
    const float & _arg)
  {
    this->range = _arg;
    return *this;
  }
  Type & set__bearing(
    const float & _arg)
  {
    this->bearing = _arg;
    return *this;
  }
  Type & set__min_range(
    const float & _arg)
  {
    this->min_range = _arg;
    return *this;
  }
  Type & set__width(
    const float & _arg)
  {
    this->width = _arg;
    return *this;
  }
  Type & set__point_count(
    const int32_t & _arg)
  {
    this->point_count = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    idaws_msgs::msg::Cluster_<ContainerAllocator> *;
  using ConstRawPtr =
    const idaws_msgs::msg::Cluster_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<idaws_msgs::msg::Cluster_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<idaws_msgs::msg::Cluster_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      idaws_msgs::msg::Cluster_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<idaws_msgs::msg::Cluster_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      idaws_msgs::msg::Cluster_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<idaws_msgs::msg::Cluster_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<idaws_msgs::msg::Cluster_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<idaws_msgs::msg::Cluster_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__idaws_msgs__msg__Cluster
    std::shared_ptr<idaws_msgs::msg::Cluster_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__idaws_msgs__msg__Cluster
    std::shared_ptr<idaws_msgs::msg::Cluster_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Cluster_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->center_y != other.center_y) {
      return false;
    }
    if (this->range != other.range) {
      return false;
    }
    if (this->bearing != other.bearing) {
      return false;
    }
    if (this->min_range != other.min_range) {
      return false;
    }
    if (this->width != other.width) {
      return false;
    }
    if (this->point_count != other.point_count) {
      return false;
    }
    return true;
  }
  bool operator!=(const Cluster_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Cluster_

// alias to use template instance with default allocator
using Cluster =
  idaws_msgs::msg::Cluster_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace idaws_msgs

#endif  // IDAWS_MSGS__MSG__DETAIL__CLUSTER__STRUCT_HPP_
