// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from yolov11_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef YOLOV11_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
#define YOLOV11_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "yolov11_msgs/msg/detail/detection__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace yolov11_msgs
{

namespace msg
{

namespace builder
{

class Init_Detection_center_y
{
public:
  explicit Init_Detection_center_y(::yolov11_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  ::yolov11_msgs::msg::Detection center_y(::yolov11_msgs::msg::Detection::_center_y_type arg)
  {
    msg_.center_y = std::move(arg);
    return std::move(msg_);
  }

private:
  ::yolov11_msgs::msg::Detection msg_;
};

class Init_Detection_center_x
{
public:
  explicit Init_Detection_center_x(::yolov11_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_center_y center_x(::yolov11_msgs::msg::Detection::_center_x_type arg)
  {
    msg_.center_x = std::move(arg);
    return Init_Detection_center_y(msg_);
  }

private:
  ::yolov11_msgs::msg::Detection msg_;
};

class Init_Detection_score
{
public:
  explicit Init_Detection_score(::yolov11_msgs::msg::Detection & msg)
  : msg_(msg)
  {}
  Init_Detection_center_x score(::yolov11_msgs::msg::Detection::_score_type arg)
  {
    msg_.score = std::move(arg);
    return Init_Detection_center_x(msg_);
  }

private:
  ::yolov11_msgs::msg::Detection msg_;
};

class Init_Detection_class_id
{
public:
  Init_Detection_class_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Detection_score class_id(::yolov11_msgs::msg::Detection::_class_id_type arg)
  {
    msg_.class_id = std::move(arg);
    return Init_Detection_score(msg_);
  }

private:
  ::yolov11_msgs::msg::Detection msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::yolov11_msgs::msg::Detection>()
{
  return yolov11_msgs::msg::builder::Init_Detection_class_id();
}

}  // namespace yolov11_msgs

#endif  // YOLOV11_MSGS__MSG__DETAIL__DETECTION__BUILDER_HPP_
