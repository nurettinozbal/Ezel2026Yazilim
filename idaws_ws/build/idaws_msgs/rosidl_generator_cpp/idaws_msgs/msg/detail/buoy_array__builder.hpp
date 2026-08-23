// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from idaws_msgs:msg/BuoyArray.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__BUILDER_HPP_
#define IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "idaws_msgs/msg/detail/buoy_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace idaws_msgs
{

namespace msg
{

namespace builder
{

class Init_BuoyArray_frame_height
{
public:
  explicit Init_BuoyArray_frame_height(::idaws_msgs::msg::BuoyArray & msg)
  : msg_(msg)
  {}
  ::idaws_msgs::msg::BuoyArray frame_height(::idaws_msgs::msg::BuoyArray::_frame_height_type arg)
  {
    msg_.frame_height = std::move(arg);
    return std::move(msg_);
  }

private:
  ::idaws_msgs::msg::BuoyArray msg_;
};

class Init_BuoyArray_frame_width
{
public:
  explicit Init_BuoyArray_frame_width(::idaws_msgs::msg::BuoyArray & msg)
  : msg_(msg)
  {}
  Init_BuoyArray_frame_height frame_width(::idaws_msgs::msg::BuoyArray::_frame_width_type arg)
  {
    msg_.frame_width = std::move(arg);
    return Init_BuoyArray_frame_height(msg_);
  }

private:
  ::idaws_msgs::msg::BuoyArray msg_;
};

class Init_BuoyArray_buoys
{
public:
  explicit Init_BuoyArray_buoys(::idaws_msgs::msg::BuoyArray & msg)
  : msg_(msg)
  {}
  Init_BuoyArray_frame_width buoys(::idaws_msgs::msg::BuoyArray::_buoys_type arg)
  {
    msg_.buoys = std::move(arg);
    return Init_BuoyArray_frame_width(msg_);
  }

private:
  ::idaws_msgs::msg::BuoyArray msg_;
};

class Init_BuoyArray_header
{
public:
  Init_BuoyArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_BuoyArray_buoys header(::idaws_msgs::msg::BuoyArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_BuoyArray_buoys(msg_);
  }

private:
  ::idaws_msgs::msg::BuoyArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::idaws_msgs::msg::BuoyArray>()
{
  return idaws_msgs::msg::builder::Init_BuoyArray_header();
}

}  // namespace idaws_msgs

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__BUILDER_HPP_
