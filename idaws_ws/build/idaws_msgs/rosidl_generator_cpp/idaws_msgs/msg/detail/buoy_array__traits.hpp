// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from idaws_msgs:msg/BuoyArray.idl
// generated code does not contain a copyright notice

#ifndef IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__TRAITS_HPP_
#define IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "idaws_msgs/msg/detail/buoy_array__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'buoys'
#include "idaws_msgs/msg/detail/buoy__traits.hpp"

namespace idaws_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const BuoyArray & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: buoys
  {
    if (msg.buoys.size() == 0) {
      out << "buoys: []";
    } else {
      out << "buoys: [";
      size_t pending_items = msg.buoys.size();
      for (auto item : msg.buoys) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: frame_width
  {
    out << "frame_width: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_width, out);
    out << ", ";
  }

  // member: frame_height
  {
    out << "frame_height: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_height, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const BuoyArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: buoys
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.buoys.size() == 0) {
      out << "buoys: []\n";
    } else {
      out << "buoys:\n";
      for (auto item : msg.buoys) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: frame_width
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_width: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_width, out);
    out << "\n";
  }

  // member: frame_height
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frame_height: ";
    rosidl_generator_traits::value_to_yaml(msg.frame_height, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const BuoyArray & msg, bool use_flow_style = false)
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
  const idaws_msgs::msg::BuoyArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  idaws_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use idaws_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const idaws_msgs::msg::BuoyArray & msg)
{
  return idaws_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<idaws_msgs::msg::BuoyArray>()
{
  return "idaws_msgs::msg::BuoyArray";
}

template<>
inline const char * name<idaws_msgs::msg::BuoyArray>()
{
  return "idaws_msgs/msg/BuoyArray";
}

template<>
struct has_fixed_size<idaws_msgs::msg::BuoyArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<idaws_msgs::msg::BuoyArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<idaws_msgs::msg::BuoyArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // IDAWS_MSGS__MSG__DETAIL__BUOY_ARRAY__TRAITS_HPP_
