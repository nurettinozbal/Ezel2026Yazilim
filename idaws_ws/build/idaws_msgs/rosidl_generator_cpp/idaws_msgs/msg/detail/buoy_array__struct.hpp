// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from idaws_msgs:msg/BuoyArray.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__STRUCT_HPP_
#define IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'buoys'
#include "idaws_msgs/msg/detail/buoy__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__idaws_msgs__msg__BuoyArray __attribute__((deprecated))
#else
# define DEPRECATED__idaws_msgs__msg__BuoyArray __declspec(deprecated)
#endif

namespace idaws_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct BuoyArray_
{
  using Type = BuoyArray_<ContainerAllocator>;

  explicit BuoyArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_width = 0l;
      this->frame_height = 0l;
    }
  }

  explicit BuoyArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->frame_width = 0l;
      this->frame_height = 0l;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _buoys_type =
    std::vector<idaws_msgs::msg::Buoy_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<idaws_msgs::msg::Buoy_<ContainerAllocator>>>;
  _buoys_type buoys;
  using _frame_width_type =
    int32_t;
  _frame_width_type frame_width;
  using _frame_height_type =
    int32_t;
  _frame_height_type frame_height;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__buoys(
    const std::vector<idaws_msgs::msg::Buoy_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<idaws_msgs::msg::Buoy_<ContainerAllocator>>> & _arg)
  {
    this->buoys = _arg;
    return *this;
  }
  Type & set__frame_width(
    const int32_t & _arg)
  {
    this->frame_width = _arg;
    return *this;
  }
  Type & set__frame_height(
    const int32_t & _arg)
  {
    this->frame_height = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    idaws_msgs::msg::BuoyArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const idaws_msgs::msg::BuoyArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<idaws_msgs::msg::BuoyArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<idaws_msgs::msg::BuoyArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      idaws_msgs::msg::BuoyArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<idaws_msgs::msg::BuoyArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      idaws_msgs::msg::BuoyArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<idaws_msgs::msg::BuoyArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<idaws_msgs::msg::BuoyArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<idaws_msgs::msg::BuoyArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__idaws_msgs__msg__BuoyArray
    std::shared_ptr<idaws_msgs::msg::BuoyArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__idaws_msgs__msg__BuoyArray
    std::shared_ptr<idaws_msgs::msg::BuoyArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const BuoyArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->buoys != other.buoys) {
      return false;
    }
    if (this->frame_width != other.frame_width) {
      return false;
    }
    if (this->frame_height != other.frame_height) {
      return false;
    }
    return true;
  }
  bool operator!=(const BuoyArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct BuoyArray_

// alias to use template instance with default allocator
using BuoyArray =
  idaws_msgs::msg::BuoyArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace idaws_msgs

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__STRUCT_HPP_
