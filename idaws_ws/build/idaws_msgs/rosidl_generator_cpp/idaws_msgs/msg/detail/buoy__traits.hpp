// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from idaws_msgs:msg/Buoy.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY__TRAITS_HPP_
#define IDAWS_MSGS__MSG__DETAIL__BUOY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "idaws_msgs/msg/detail/buoy__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace idaws_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Buoy & msg,
  std::ostream & out)
{
  out << "{";
  // member: label
  {
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << ", ";
  }

  // member: confidence
  {
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << ", ";
  }

  // member: x_min
  {
    out << "x_min: ";
    rosidl_generator_traits::value_to_yaml(msg.x_min, out);
    out << ", ";
  }

  // member: y_min
  {
    out << "y_min: ";
    rosidl_generator_traits::value_to_yaml(msg.y_min, out);
    out << ", ";
  }

  // member: x_max
  {
    out << "x_max: ";
    rosidl_generator_traits::value_to_yaml(msg.x_max, out);
    out << ", ";
  }

  // member: y_max
  {
    out << "y_max: ";
    rosidl_generator_traits::value_to_yaml(msg.y_max, out);
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
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Buoy & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: label
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "label: ";
    rosidl_generator_traits::value_to_yaml(msg.label, out);
    out << "\n";
  }

  // member: confidence
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "confidence: ";
    rosidl_generator_traits::value_to_yaml(msg.confidence, out);
    out << "\n";
  }

  // member: x_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_min: ";
    rosidl_generator_traits::value_to_yaml(msg.x_min, out);
    out << "\n";
  }

  // member: y_min
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_min: ";
    rosidl_generator_traits::value_to_yaml(msg.y_min, out);
    out << "\n";
  }

  // member: x_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "x_max: ";
    rosidl_generator_traits::value_to_yaml(msg.x_max, out);
    out << "\n";
  }

  // member: y_max
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "y_max: ";
    rosidl_generator_traits::value_to_yaml(msg.y_max, out);
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
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Buoy & msg, bool use_flow_style = false)
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
  const idaws_msgs::msg::Buoy & msg,
  std::ostream & out, size_t indentation = 0)
{
  idaws_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use idaws_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const idaws_msgs::msg::Buoy & msg)
{
  return idaws_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<idaws_msgs::msg::Buoy>()
{
  return "idaws_msgs::msg::Buoy";
}

template<>
inline const char * name<idaws_msgs::msg::Buoy>()
{
  return "idaws_msgs/msg/Buoy";
}

template<>
struct has_fixed_size<idaws_msgs::msg::Buoy>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<idaws_msgs::msg::Buoy>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<idaws_msgs::msg::Buoy>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY__TRAITS_HPP_
