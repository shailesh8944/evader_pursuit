// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:msg/Actuator.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__MSG__DETAIL__ACTUATOR__STRUCT_H_
#define INTERFACES__MSG__DETAIL__ACTUATOR__STRUCT_H_

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
// Member 'covariance'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Actuator in the package interfaces.
typedef struct interfaces__msg__Actuator
{
  std_msgs__msg__Header header;
  /// Single rudder vessel fields
  double rudder;
  double propeller;
  /// MAVYMINI control surfaces
  double upper_rudder;
  double lower_rudder;
  double stern_fin_left;
  double stern_fin_right;
  double bow_fin_left;
  double bow_fin_right;
  /// Covariance matrix (flattened)
  /// For single rudder: 2x2 matrix = 4 elements
  /// For MAVYMINI: 7x7 matrix = 49 elements (6 surfaces + 1 propeller)
  rosidl_runtime_c__double__Sequence covariance;
} interfaces__msg__Actuator;

// Struct for a sequence of interfaces__msg__Actuator.
typedef struct interfaces__msg__Actuator__Sequence
{
  interfaces__msg__Actuator * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__msg__Actuator__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__MSG__DETAIL__ACTUATOR__STRUCT_H_
