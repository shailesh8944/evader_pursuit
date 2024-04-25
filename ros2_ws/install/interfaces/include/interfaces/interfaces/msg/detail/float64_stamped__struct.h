// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Float64Stamped.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__FLOAT64_STAMPED__STRUCT_H_
#define INTERFACES__MSG__DETAIL__FLOAT64_STAMPED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/Float64Stamped in the package interfaces.
typedef struct interfaces__msg__Float64Stamped
{
  std_msgs__msg__Header header;
  double value;
} interfaces__msg__Float64Stamped;

// Struct for a sequence of interfaces__msg__Float64Stamped.
typedef struct interfaces__msg__Float64Stamped__Sequence
{
  interfaces__msg__Float64Stamped * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Float64Stamped__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__FLOAT64_STAMPED__STRUCT_H_
