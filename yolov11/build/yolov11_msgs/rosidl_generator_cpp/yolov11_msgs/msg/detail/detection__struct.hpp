// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from yolov11_msgs:msg/Detection.idl
// generated code does not contain a copyright notice

#ifndef YOLOV11_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_
#define YOLOV11_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__yolov11_msgs__msg__Detection __attribute__((deprecated))
#else
# define DEPRECATED__yolov11_msgs__msg__Detection __declspec(deprecated)
#endif

namespace yolov11_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Detection_
{
  using Type = Detection_<ContainerAllocator>;

  explicit Detection_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = "";
      this->score = 0.0f;
      this->center_x = 0l;
      this->center_y = 0l;
    }
  }

  explicit Detection_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : class_id(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->class_id = "";
      this->score = 0.0f;
      this->center_x = 0l;
      this->center_y = 0l;
    }
  }

  // field types and members
  using _class_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _class_id_type class_id;
  using _score_type =
    float;
  _score_type score;
  using _center_x_type =
    int32_t;
  _center_x_type center_x;
  using _center_y_type =
    int32_t;
  _center_y_type center_y;

  // setters for named parameter idiom
  Type & set__class_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->class_id = _arg;
    return *this;
  }
  Type & set__score(
    const float & _arg)
  {
    this->score = _arg;
    return *this;
  }
  Type & set__center_x(
    const int32_t & _arg)
  {
    this->center_x = _arg;
    return *this;
  }
  Type & set__center_y(
    const int32_t & _arg)
  {
    this->center_y = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    yolov11_msgs::msg::Detection_<ContainerAllocator> *;
  using ConstRawPtr =
    const yolov11_msgs::msg::Detection_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<yolov11_msgs::msg::Detection_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<yolov11_msgs::msg::Detection_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      yolov11_msgs::msg::Detection_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<yolov11_msgs::msg::Detection_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      yolov11_msgs::msg::Detection_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<yolov11_msgs::msg::Detection_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<yolov11_msgs::msg::Detection_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<yolov11_msgs::msg::Detection_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__yolov11_msgs__msg__Detection
    std::shared_ptr<yolov11_msgs::msg::Detection_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__yolov11_msgs__msg__Detection
    std::shared_ptr<yolov11_msgs::msg::Detection_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Detection_ & other) const
  {
    if (this->class_id != other.class_id) {
      return false;
    }
    if (this->score != other.score) {
      return false;
    }
    if (this->center_x != other.center_x) {
      return false;
    }
    if (this->center_y != other.center_y) {
      return false;
    }
    return true;
  }
  bool operator!=(const Detection_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Detection_

// alias to use template instance with default allocator
using Detection =
  yolov11_msgs::msg::Detection_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace yolov11_msgs

#endif  // YOLOV11_MSGS__MSG__DETAIL__DETECTION__STRUCT_HPP_
