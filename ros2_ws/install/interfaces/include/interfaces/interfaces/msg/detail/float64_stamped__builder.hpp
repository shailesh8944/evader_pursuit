// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Float64Stamped.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__FLOAT64_STAMPED__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__FLOAT64_STAMPED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/float64_stamped__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Float64Stamped_value
{
public:
  explicit Init_Float64Stamped_value(::interfaces::msg::Float64Stamped & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Float64Stamped value(::interfaces::msg::Float64Stamped::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Float64Stamped msg_;
};

class Init_Float64Stamped_header
{
public:
  Init_Float64Stamped_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Float64Stamped_value header(::interfaces::msg::Float64Stamped::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Float64Stamped_value(msg_);
  }

private:
  ::interfaces::msg::Float64Stamped msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Float64Stamped>()
{
  return interfaces::msg::builder::Init_Float64Stamped_header();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__FLOAT64_STAMPED__BUILDER_HPP_
