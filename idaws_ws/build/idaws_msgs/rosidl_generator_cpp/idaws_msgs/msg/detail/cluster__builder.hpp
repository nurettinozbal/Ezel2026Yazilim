// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from idaws_msgs:msg/Cluster.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__CLUSTER__BUILDER_HPP_
#define IDAWS_MSGS__MSG__DETAIL__CLUSTER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "idaws_msgs/msg/detail/cluster__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace idaws_msgs
{

namespace msg
{

namespace builder
{

class Init_Cluster_point_count
{
public:
  explicit Init_Cluster_point_count(::idaws_msgs::msg::Cluster & msg)
  : msg_(msg)
  {}
  ::idaws_msgs::msg::Cluster point_count(::idaws_msgs::msg::Cluster::_point_count_type arg)
  {
    msg_.point_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::idaws_msgs::msg::Cluster msg_;
};

class Init_Cluster_width
{
public:
  explicit Init_Cluster_width(::idaws_msgs::msg::Cluster & msg)
  : msg_(msg)
  {}
  Init_Cluster_point_count width(::idaws_msgs::msg::Cluster::_width_type arg)
  {
    msg_.width = std::move(arg);
    return Init_Cluster_point_count(msg_);
  }

private:
  ::idaws_msgs::msg::Cluster msg_;
};

class Init_Cluster_min_range
{
public:
  explicit Init_Cluster_min_range(::idaws_msgs::msg::Cluster & msg)
  : msg_(msg)
  {}
  Init_Cluster_width min_range(::idaws_msgs::msg::Cluster::_min_range_type arg)
  {
    msg_.min_range = std::move(arg);
    return Init_Cluster_width(msg_);
  }

private:
  ::idaws_msgs::msg::Cluster msg_;
};

class Init_Cluster_bearing
{
public:
  explicit Init_Cluster_bearing(::idaws_msgs::msg::Cluster & msg)
  : msg_(msg)
  {}
  Init_Cluster_min_range bearing(::idaws_msgs::msg::Cluster::_bearing_type arg)
  {
    msg_.bearing = std::move(arg);
    return Init_Cluster_min_range(msg_);
  }

private:
  ::idaws_msgs::msg::Cluster msg_;
};

class Init_Cluster_range
{
public:
  explicit Init_Cluster_range(::idaws_msgs::msg::Cluster & msg)
  : msg_(msg)
  {}
  Init_Cluster_bearing range(::idaws_msgs::msg::Cluster::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_Cluster_bearing(msg_);
  }

private:
  ::idaws_msgs::msg::Cluster msg_;
};

class Init_Cluster_center_y
{
public:
  explicit Init_Cluster_center_y(::idaws_msgs::msg::Cluster & msg)
  : msg_(msg)
  {}
  Init_Cluster_range center_y(::idaws_msgs::msg::Cluster::_center_y_type arg)
  {
    msg_.center_y = std::move(arg);
    return Init_Cluster_range(msg_);
  }

private:
  ::idaws_msgs::msg::Cluster msg_;
};

class Init_Cluster_center_x
{
public:
  explicit Init_Cluster_center_x(::idaws_msgs::msg::Cluster & msg)
  : msg_(msg)
  {}
  Init_Cluster_center_y center_x(::idaws_msgs::msg::Cluster::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_Cluster_center_y(msg_);
  }

private:
  ::idaws_msgs::msg::Cluster msg_;
};

class Init_Cluster_id
{
public:
  Init_Cluster_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cluster_center_x id(::idaws_msgs::msg::Cluster::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Cluster_center_x(msg_);
  }

private:
  ::idaws_msgs::msg::Cluster msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::idaws_msgs::msg::Cluster>()
{
  return idaws_msgs::msg::builder::Init_Cluster_id();
}

}  // namespace idaws_msgs

#endif  // IDAWS_MSGS__MSG__DETAIL__CLUSTER__BUILDER_HPP_
