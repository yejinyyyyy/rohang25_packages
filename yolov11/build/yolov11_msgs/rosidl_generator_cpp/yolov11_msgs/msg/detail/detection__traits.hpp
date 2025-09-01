// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from yolov11_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef YOLOV11_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_
#define YOLOV11_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "yolov11_msgs/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace yolov11_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Detection & msg,
  std::ostream & out)
{
  out << "{";
  // member: class_id
  {
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << ", ";
  }

  // member: score
  {
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
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
  const Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: class_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "class_id: ";
    rosidl_generator_traits::value_to_yaml(msg.class_id, out);
    out << "\n";
  }

  // member: score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "score: ";
    rosidl_generator_traits::value_to_yaml(msg.score, out);
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

inline std::string to_yaml(const Detection & msg, bool use_flow_style = false)
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

}  // namespace yolov11_msgs

namespace rosidl_generator_traits
{

[[deprecated("use yolov11_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const yolov11_msgs::msg::Detection & msg,
  std::ostream & out, size_t indentation = 0)
{
  yolov11_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use yolov11_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const yolov11_msgs::msg::Detection & msg)
{
  return yolov11_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<yolov11_msgs::msg::Detection>()
{
  return "yolov11_msgs::msg::Detection";
}

template<>
inline const char * name<yolov11_msgs::msg::Detection>()
{
  return "yolov11_msgs/msg/Detection";
}

template<>
struct has_fixed_size<yolov11_msgs::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<yolov11_msgs::msg::Detection>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<yolov11_msgs::msg::Detection>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // YOLOV11_MSGS__MSG__DETAIL__DETECTION__TRAITS_HPP_
