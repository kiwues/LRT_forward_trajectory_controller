// Copyright (c) 2022 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_
#define JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_

#include <string>
#include <vector>

#include "parameter_traits/parameter_traits.hpp"
#include "rclcpp/parameter.hpp"
#include "rsl/algorithm.hpp"
#include "tl_expected/expected.hpp"

namespace joint_trajectory_controller
{
tl::expected<void, std::string> command_interface_type_combinations(
  rclcpp::Parameter const & parameter)
{
  auto const & interface_types = parameter.as_string_array();

  // Check if command interfaces combination is valid. Valid combinations are:
  // 1. effort
  // 2. velocity
  // 2. position [velocity, [acceleration]]

  // zakomnetowuje orginał
/*
  if (
    rsl::contains<std::vector<std::string>>(interface_types, "velocity") &&
    interface_types.size() > 1 &&
    !rsl::contains<std::vector<std::string>>(interface_types, "position"))
  {
    return tl::make_unexpected(
      "'velocity' command interface can be used either alone or 'position' "
      "command interface has to be present");
  }

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "acceleration") &&
    (!rsl::contains<std::vector<std::string>>(interface_types, "velocity") &&
     !rsl::contains<std::vector<std::string>>(interface_types, "position")))
  {
    return tl::make_unexpected(
      "'acceleration' command interface can only be used if 'velocity' and "
      "'position' command interfaces are present");
  }

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "effort") &&
    interface_types.size() > 1)
  {
    return tl::make_unexpected("'effort' command interface has to be used alone");
  }

  return {};*/
  
  // Allowed command interface combinations (today's goal):
  // 1) position
  // 2) position + velocity
  // 3) position + effort
  // 4) position + velocity + effort
  //
  // Not allowed:
  // - acceleration in command interfaces
  // - velocity without position
  // - effort without position

  const bool has_position =
    rsl::contains<std::vector<std::string>>(interface_types, "position");
  const bool has_velocity =
    rsl::contains<std::vector<std::string>>(interface_types, "velocity");
  const bool has_acceleration =
    rsl::contains<std::vector<std::string>>(interface_types, "acceleration");
  const bool has_effort =
    rsl::contains<std::vector<std::string>>(interface_types, "effort");

  if (!has_position)
  {
    return tl::make_unexpected("'position' command interface is required.");
  }

  if (has_acceleration)
  {
    return tl::make_unexpected(
      "'acceleration' command interface is not allowed in this controller configuration.");
  }

  // At this point we know: position is present, acceleration is not present.
  // Only velocity and/or effort may appear in addition.

  if (interface_types.size() == 1)
  {
    // Only "position"
    return {};
  }

  if (interface_types.size() == 2)
  {
    // Must be either {position, velocity} or {position, effort}
    if ((has_velocity && !has_effort) || (!has_velocity && has_effort))
    {
      return {};
    }
    return tl::make_unexpected(
      "Invalid command interfaces. Allowed: [position], [position, velocity], "
      "[position, effort], [position, velocity, effort].");
  }

  if (interface_types.size() == 3)
  {
    // Must be {position, velocity, effort}
    if (has_velocity && has_effort)
    {
      return {};
    }
    return tl::make_unexpected(
      "Invalid command interfaces. Allowed: [position], [position, velocity], "
      "[position, effort], [position, velocity, effort].");
  }

  return tl::make_unexpected(
    "Invalid number of command interfaces. Allowed: 1, 2, or 3.");
  
  // od komentarza o zakomentowaniu do tego miejsca jest nowy, dodany kod
}

// orginalny zakometowany kod
/*
tl::expected<void, std::string> state_interface_type_combinations(
  rclcpp::Parameter const & parameter)
{
  auto const & interface_types = parameter.as_string_array();

  // Valid combinations are
  // 1. position [velocity, [acceleration]]

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "velocity") &&
    !rsl::contains<std::vector<std::string>>(interface_types, "position"))
  {
    return tl::make_unexpected(
      "'velocity' state interface cannot be used if 'position' interface "
      "is missing.");
  }

  if (
    rsl::contains<std::vector<std::string>>(interface_types, "acceleration") &&
    (!rsl::contains<std::vector<std::string>>(interface_types, "position") ||
     !rsl::contains<std::vector<std::string>>(interface_types, "velocity")))
  {
    return tl::make_unexpected(
      "'acceleration' state interface cannot be used if 'position' and 'velocity' "
      "interfaces are not present.");
  }

  return {};
} */

//dodany nowy kod

tl::expected<void, std::string> state_interface_type_combinations(
  rclcpp::Parameter const & parameter)
{
  auto const & interface_types = parameter.as_string_array();

  // Today's goal: fixed state interfaces = exactly [position, velocity, effort]
  const bool has_position =
    rsl::contains<std::vector<std::string>>(interface_types, "position");
  const bool has_velocity =
    rsl::contains<std::vector<std::string>>(interface_types, "velocity");
  const bool has_effort =
    rsl::contains<std::vector<std::string>>(interface_types, "effort");
  const bool has_acceleration =
    rsl::contains<std::vector<std::string>>(interface_types, "acceleration");

  if (has_acceleration)
  {
    return tl::make_unexpected(
      "'acceleration' state interface is not allowed. Required: [position, velocity, effort].");
  }

  if (!has_position || !has_velocity || !has_effort)
  {
    return tl::make_unexpected(
      "State interfaces must contain exactly: [position, velocity, effort].");
  }

  if (interface_types.size() != 3)
  {
    return tl::make_unexpected(
      "Invalid number of state interfaces. Required exactly 3: [position, velocity, effort].");
  }

  return {};
}

// koniec dodanego kodu

}  // namespace joint_trajectory_controller

#endif  // JOINT_TRAJECTORY_CONTROLLER__VALIDATE_JTC_PARAMETERS_HPP_
