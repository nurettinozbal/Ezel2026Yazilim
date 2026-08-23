// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from idaws_msgs:msg/Buoy.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY__BUILDER_HPP_
#define IDAWS_MSGS__MSG__DETAIL__BUOY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "idaws_msgs/msg/detail/buoy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace idaws_msgs
{

namespace msg
{

namespace builder
{

class Init_Buoy_center_y
{
public:
  explicit Init_Buoy_center_y(::idaws_msgs::msg::Buoy & msg)
  : msg_(msg)
  {}
  ::idaws_msgs::msg::Buoy center_y(::idaws_msgs::msg::Buoy::_center_y_type arg)
  {
    msg_.center_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::idaws_msgs::msg::Buoy msg_;
};

class Init_Buoy_center_x
{
public:
  explicit Init_Buoy_center_x(::idaws_msgs::msg::Buoy & msg)
  : msg_(msg)
  {}
  Init_Buoy_center_y center_x(::idaws_msgs::msg::Buoy::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_Buoy_center_y(msg_);
  }

private:
  ::idaws_msgs::msg::Buoy msg_;
};

class Init_Buoy_y_max
{
public:
  explicit Init_Buoy_y_max(::idaws_msgs::msg::Buoy & msg)
  : msg_(msg)
  {}
  Init_Buoy_center_x y_max(::idaws_msgs::msg::Buoy::_y_max_type arg)
  {
    msg_.y_max = std::move(arg);
    return Init_Buoy_center_x(msg_);
  }

private:
  ::idaws_msgs::msg::Buoy msg_;
};

class Init_Buoy_x_max
{
public:
  explicit Init_Buoy_x_max(::idaws_msgs::msg::Buoy & msg)
  : msg_(msg)
  {}
  Init_Buoy_y_max x_max(::idaws_msgs::msg::Buoy::_x_max_type arg)
  {
    msg_.x_max = std::move(arg);
    return Init_Buoy_y_max(msg_);
  }

private:
  ::idaws_msgs::msg::Buoy msg_;
};

class Init_Buoy_y_min
{
public:
  explicit Init_Buoy_y_min(::idaws_msgs::msg::Buoy & msg)
  : msg_(msg)
  {}
  Init_Buoy_x_max y_min(::idaws_msgs::msg::Buoy::_y_min_type arg)
  {
    msg_.y_min = std::move(arg);
    return Init_Buoy_x_max(msg_);
  }

private:
  ::idaws_msgs::msg::Buoy msg_;
};

class Init_Buoy_x_min
{
public:
  explicit Init_Buoy_x_min(::idaws_msgs::msg::Buoy & msg)
  : msg_(msg)
  {}
  Init_Buoy_y_min x_min(::idaws_msgs::msg::Buoy::_x_min_type arg)
  {
    msg_.x_min = std::move(arg);
    return Init_Buoy_y_min(msg_);
  }

private:
  ::idaws_msgs::msg::Buoy msg_;
};

class Init_Buoy_confidence
{
public:
  explicit Init_Buoy_confidence(::idaws_msgs::msg::Buoy & msg)
  : msg_(msg)
  {}
  Init_Buoy_x_min confidence(::idaws_msgs::msg::Buoy::_confidence_type arg)
  {
    msg_.confidence = std::move(arg);
    return Init_Buoy_x_min(msg_);
  }

private:
  ::idaws_msgs::msg::Buoy msg_;
};

class Init_Buoy_label
{
public:
  Init_Buoy_label()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Buoy_confidence label(::idaws_msgs::msg::Buoy::_label_type arg)
  {
    msg_.label = std::move(arg);
    return Init_Buoy_confidence(msg_);
  }

private:
  ::idaws_msgs::msg::Buoy msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::idaws_msgs::msg::Buoy>()
{
  return idaws_msgs::msg::builder::Init_Buoy_label();
}

}  // namespace idaws_msgs

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY__BUILDER_HPP_
