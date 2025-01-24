// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:msg/Actuator.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ACTUATOR__BUILDER_HPP_
#define INTERFACES__MSG__DETAIL__ACTUATOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/msg/detail/actuator__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace msg
{

namespace builder
{

class Init_Actuator_covariance
{
public:
  explicit Init_Actuator_covariance(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  ::interfaces::msg::Actuator covariance(::interfaces::msg::Actuator::_covariance_type arg)
  {
    msg_.covariance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_bow_fin_right
{
public:
  explicit Init_Actuator_bow_fin_right(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_covariance bow_fin_right(::interfaces::msg::Actuator::_bow_fin_right_type arg)
  {
    msg_.bow_fin_right = std::move(arg);
    return Init_Actuator_covariance(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_bow_fin_left
{
public:
  explicit Init_Actuator_bow_fin_left(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_bow_fin_right bow_fin_left(::interfaces::msg::Actuator::_bow_fin_left_type arg)
  {
    msg_.bow_fin_left = std::move(arg);
    return Init_Actuator_bow_fin_right(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_stern_fin_right
{
public:
  explicit Init_Actuator_stern_fin_right(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_bow_fin_left stern_fin_right(::interfaces::msg::Actuator::_stern_fin_right_type arg)
  {
    msg_.stern_fin_right = std::move(arg);
    return Init_Actuator_bow_fin_left(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_stern_fin_left
{
public:
  explicit Init_Actuator_stern_fin_left(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_stern_fin_right stern_fin_left(::interfaces::msg::Actuator::_stern_fin_left_type arg)
  {
    msg_.stern_fin_left = std::move(arg);
    return Init_Actuator_stern_fin_right(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_lower_rudder
{
public:
  explicit Init_Actuator_lower_rudder(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_stern_fin_left lower_rudder(::interfaces::msg::Actuator::_lower_rudder_type arg)
  {
    msg_.lower_rudder = std::move(arg);
    return Init_Actuator_stern_fin_left(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_upper_rudder
{
public:
  explicit Init_Actuator_upper_rudder(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_lower_rudder upper_rudder(::interfaces::msg::Actuator::_upper_rudder_type arg)
  {
    msg_.upper_rudder = std::move(arg);
    return Init_Actuator_lower_rudder(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_propeller
{
public:
  explicit Init_Actuator_propeller(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_upper_rudder propeller(::interfaces::msg::Actuator::_propeller_type arg)
  {
    msg_.propeller = std::move(arg);
    return Init_Actuator_upper_rudder(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_rudder
{
public:
  explicit Init_Actuator_rudder(::interfaces::msg::Actuator & msg)
  : msg_(msg)
  {}
  Init_Actuator_propeller rudder(::interfaces::msg::Actuator::_rudder_type arg)
  {
    msg_.rudder = std::move(arg);
    return Init_Actuator_propeller(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

class Init_Actuator_header
{
public:
  Init_Actuator_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Actuator_rudder header(::interfaces::msg::Actuator::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Actuator_rudder(msg_);
  }

private:
  ::interfaces::msg::Actuator msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::msg::Actuator>()
{
  return interfaces::msg::builder::Init_Actuator_header();
}

}  // namespace interfaces

#endif  // INTERFACES__MSG__DETAIL__ACTUATOR__BUILDER_HPP_
