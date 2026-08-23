// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from idaws_msgs:msg/Cluster.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__CLUSTER__TRAITS_HPP_
#define IDAWS_MSGS__MSG__DETAIL__CLUSTER__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "idaws_msgs/msg/detail/cluster__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace idaws_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Cluster & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: center_x
  {
    out << "center_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_x, out);
    out << ", ";
  }

  // member: center_y
  {
    out << "center_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_y, out);
    out << ", ";
  }

  // member: range
  {
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << ", ";
  }

  // member: bearing
  {
    out << "bearing: ";
    rosidl_generator_traits::value_to_yaml(msg.bearing, out);
    out << ", ";
  }

  // member: min_range
  {
    out << "min_range: ";
    rosidl_generator_traits::value_to_yaml(msg.min_range, out);
    out << ", ";
  }

  // member: width
  {
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << ", ";
  }

  // member: point_count
  {
    out << "point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.point_count, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Cluster & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: center_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_x: ";
    rosidl_generator_traits::value_to_yaml(msg.center_x, out);
    out << "\n";
  }

  // member: center_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "center_y: ";
    rosidl_generator_traits::value_to_yaml(msg.center_y, out);
    out << "\n";
  }

  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << "\n";
  }

  // member: bearing
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bearing: ";
    rosidl_generator_traits::value_to_yaml(msg.bearing, out);
    out << "\n";
  }

  // member: min_range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_range: ";
    rosidl_generator_traits::value_to_yaml(msg.min_range, out);
    out << "\n";
  }

  // member: width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "width: ";
    rosidl_generator_traits::value_to_yaml(msg.width, out);
    out << "\n";
  }

  // member: point_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point_count: ";
    rosidl_generator_traits::value_to_yaml(msg.point_count, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Cluster & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace idaws_msgs

namespace rosidl_generator_traits
{

[[deprecated("use idaws_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const idaws_msgs::msg::Cluster & msg,
  std::ostream & out, size_t indentation = 0)
{
  idaws_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use idaws_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const idaws_msgs::msg::Cluster & msg)
{
  return idaws_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<idaws_msgs::msg::Cluster>()
{
  return "idaws_msgs::msg::Cluster";
}

template<>
inline const char * name<idaws_msgs::msg::Cluster>()
{
  return "idaws_msgs/msg/Cluster";
}

template<>
struct has_fixed_size<idaws_msgs::msg::Cluster>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<idaws_msgs::msg::Cluster>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<idaws_msgs::msg::Cluster>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // IDAWS_MSGS__MSG__DETAIL__CLUSTER__TRAITS_HPP_
