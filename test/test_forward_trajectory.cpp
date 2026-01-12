// Copyright 2024 LRT
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

#include "gtest/gtest.h"
#include "test_trajectory_controller_utils.hpp"

using lifecycle_msgs::msg::State;
using test_trajectory_controllers::TrajectoryControllerTest;
using test_trajectory_controllers::TrajectoryControllerTestParameterized;

namespace
{
constexpr double EPS = 0.001;
constexpr int NUM_SAMPLES = 100;
}  // namespace

//=============================================================================
// Custom test fixture with logging capabilities
//=============================================================================
class TrajectoryInterpolationTest : public TrajectoryControllerTest
{
public:
  void SetUp() override
  {
    TrajectoryControllerTest::SetUp();
    logged_data_.clear();
  }

  struct TrajectoryLogEntry
  {
    double time_sec;
    std::vector<double> reference_positions;
    std::vector<double> reference_velocities;
    std::vector<double> reference_accelerations;
    std::vector<double> reference_efforts;
    std::vector<double> feedback_positions;
    std::vector<double> feedback_velocities;
    std::vector<double> feedback_efforts;
    std:: vector<double> command_positions;
    std::vector<double> command_velocities;
    std::vector<double> command_efforts;
    std:: vector<double> error_positions;
    std::vector<double> error_velocities;
  };

  std::vector<TrajectoryLogEntry> logged_data_;
    std::vector<std::vector<double>> input_positions_;
    std::vector<std::vector<double>> input_velocities_;
    std::vector<std::vector<double>> input_efforts_;
    std::vector<double> input_times_;
    std::string test_config_name_;
    std::string interpolation_method_;

  void log_current_state(double time_sec)
  {
    TrajectoryLogEntry entry;
    entry.time_sec = time_sec;

    auto state_ref = traj_controller_->get_state_reference();
    auto state_fb = traj_controller_->get_state_feedback();
    auto state_err = traj_controller_->get_state_error();

    entry.reference_positions = state_ref.positions;
    entry.reference_velocities = state_ref.velocities;
    entry.reference_accelerations = state_ref.accelerations;
    entry.reference_efforts = state_ref.effort;

    entry.feedback_positions = state_fb.positions;
    entry.feedback_velocities = state_fb.velocities;
    entry.feedback_efforts = state_fb.effort;

    entry.command_positions = joint_pos_;
    entry.command_velocities = joint_vel_;
    entry.command_efforts = joint_eff_;

    entry.error_positions = state_err.positions;
    entry.error_velocities = state_err.velocities;

    logged_data_.push_back(entry);
  }

  void print_log_header()
  {
    std::cout << "\n";
    std::cout << "========================================\n";
    std:: cout << "=== TRAJECTORY INTERPOLATION LOG =======\n";
    std::cout << "========================================\n";
    std::cout << "CONFIG:  " << test_config_name_ << "\n";
    std:: cout << "INTERPOLATION: " << interpolation_method_ << "\n";
    std::cout << "========================================\n";
  }

  void print_input_trajectory()
  {
    std::cout << "--- INPUT_TRAJECTORY_START ---\n";
    std::cout << "NUM_JOINTS: " << joint_names_.size() << "\n";
    std::cout << "JOINT_NAMES: ";
    for (size_t i = 0; i < joint_names_.size(); ++i)
    {
      if (i > 0) std::cout << ",";
      std::cout << joint_names_[i];
    }
    std::cout << "\n";

    std::cout << "NUM_POINTS: " << input_positions_.size() << "\n";

    for (size_t p = 0; p < input_positions_.size(); ++p)
    {
      std::cout << "POINT " << p << ":\n";
      std::cout << "  TIME: " << std::fixed << std::setprecision(4) << input_times_[p] << "\n";

      std::cout << "  POSITIONS:  ";
      for (size_t j = 0; j < input_positions_[p].size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std:: cout << std::fixed << std::setprecision(6) << input_positions_[p][j];
      }
      std::cout << "\n";

      if (p < input_velocities_.size() && !input_velocities_[p].empty())
      {
        std::cout << "  VELOCITIES: ";
        for (size_t j = 0; j < input_velocities_[p].size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << input_velocities_[p][j];
        }
        std::cout << "\n";
      }

      if (p < input_efforts_.size() && !input_efforts_[p].empty())
      {
        std::cout << "  EFFORTS: ";
        for (size_t j = 0; j < input_efforts_[p].size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << input_efforts_[p][j];
        }
        std::cout << "\n";
      }
    }
    std::cout << "--- INPUT_TRAJECTORY_END ---\n";
  }

  void print_sampled_trajectory()
  {
    std::cout << "--- SAMPLED_TRAJECTORY_START ---\n";
    std::cout << "NUM_SAMPLES: " << logged_data_.size() << "\n";

    for (size_t s = 0; s < logged_data_.size(); ++s)
    {
      const auto & entry = logged_data_[s];
      std::cout << "SAMPLE " << s << ":\n";
      std::cout << "  TIME: " << std:: fixed << std::setprecision(4) << entry.time_sec << "\n";

      // Reference position
      std::cout << "  POS: ";
      for (size_t j = 0; j < entry.reference_positions.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.reference_positions[j];
      }
      std::cout << "\n";

      // Reference velocity
      if (!entry.reference_velocities.empty())
      {
        std::cout << "  VEL: ";
        for (size_t j = 0; j < entry.reference_velocities.size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << entry.reference_velocities[j];
        }
        std::cout << "\n";
      }

      // Reference acceleration
      if (!entry.reference_accelerations.empty())
      {
        std::cout << "  ACC: ";
        for (size_t j = 0; j < entry.reference_accelerations.size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << entry.reference_accelerations[j];
        }
        std::cout << "\n";
      }

      // Reference effort
      if (!entry.reference_efforts.empty())
      {
        std::cout << "  EFF: ";
        for (size_t j = 0; j < entry.reference_efforts.size(); ++j)
        {
          if (j > 0) std::cout << ",";
          std::cout << std::fixed << std::setprecision(6) << entry.reference_efforts[j];
        }
        std:: cout << "\n";
      }

      // Command position
      std::cout << "  CMD_POS: ";
      for (size_t j = 0; j < entry.command_positions.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.command_positions[j];
      }
      std::cout << "\n";

      // Command velocity
      std::cout << "  CMD_VEL: ";
      for (size_t j = 0; j < entry.command_velocities.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std:: fixed << std::setprecision(6) << entry.command_velocities[j];
      }
      std::cout << "\n";

      // Command effort
      std::cout << "  CMD_EFF: ";
      for (size_t j = 0; j < entry.command_efforts.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.command_efforts[j];
      }
      std::cout << "\n";

      // Feedback position
      std::cout << "  FB_POS: ";
      for (size_t j = 0; j < entry.feedback_positions.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.feedback_positions[j];
      }
      std::cout << "\n";

      // Position error
      std::cout << "  ERR_POS: ";
      for (size_t j = 0; j < entry.error_positions.size(); ++j)
      {
        if (j > 0) std::cout << ",";
        std::cout << std::fixed << std::setprecision(6) << entry.error_positions[j];
      }
      std::cout << "\n";
    }
    std::cout << "--- SAMPLED_TRAJECTORY_END ---\n";
  }

  void print_log_footer()
  {
    std::cout << "========================================\n";
    std::cout << "=== END LOG ============================\n";
    std:: cout << "========================================\n\n";
  }

  void run_trajectory_with_logging(
    rclcpp:: Executor & executor,
    double total_duration_sec,
    int num_samples)
  {
    double dt = total_duration_sec / num_samples;
    rclcpp::Time start_time(0, 0, RCL_STEADY_TIME);
    rclcpp::Duration update_period = rclcpp::Duration:: from_seconds(0.001);

    for (int i = 0; i <= num_samples; ++i)
    {
      double current_time = i * dt;
      rclcpp::Time sample_time = start_time + rclcpp::Duration:: from_seconds(current_time);

      traj_controller_->update(sample_time, update_period);
      log_current_state(current_time);
    }
  }
};

//=============================================================================
// TEST CASE 1: Position + Effort
// Input: position, effort -> Output: position + velocity + acceleration + effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, case1_position_effort)
{
  // Define consistent trajectory waypoints with 5 points
  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.3, 0.3, 0.3},
    {1.0, 1.0, 1.0},
    {1.5, 1.5, 1.5},
    {2.0, 2.0, 2.0}
  };
  input_velocities_ = {};  // No velocities specified
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {2.0, 2.0, 2.0},
    {5.0, 5.0, 5.0},
    {3.0, 3.0, 3.0},
    {0.0, 0.0, 0.0}
  };

  command_interface_types_ = {"position", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "case1_position_effort";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.effort = input_efforts_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  EXPECT_NEAR(2.0, logged_data_.back().command_positions[0], EPS);
  executor.cancel();
}

//=============================================================================
// TEST CASE 2: Position + Velocity + Effort
// Input: position, velocity, effort -> Output: position + velocity + acceleration + effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, case2_position_velocity_effort)
{
  // Define consistent trajectory waypoints with 5 points
  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.3, 0.3, 0.3},
    {1.0, 1.0, 1.0},
    {1.5, 1.5, 1.5},
    {2.0, 2.0, 2.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {1.5, 1.5, 1.5},  // Specified velocity
    {2.0, 2.0, 2.0},
    {1.0, 1.0, 1.0},
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {2.0, 2.0, 2.0},
    {5.0, 5.0, 5.0},
    {3.0, 3.0, 3.0},
    {0.0, 0.0, 0.0}
  };

  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "case2_position_velocity_effort";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);
  SetUpAndActivateTrajectoryController(executor, params);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.effort = input_efforts_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();

}

//=============================================================================
// TEST CASE 3: Velocity + Effort (Position calculated)
// Input: velocity, effort -> Output: position + velocity + acceleration + effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, case3_velocity_effort)
{
  // Define consistent trajectory waypoints with 5 points
  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.3, 0.3, 0.3},
    {1.0, 1.0, 1.0},
    {1.5, 1.5, 1.5},
    {2.0, 2.0, 2.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {1.5, 1.5, 1.5},
    {2.0, 2.0, 2.0},
    {1.0, 1.0, 1.0},
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {2.0, 2.0, 2.0},
    {5.0, 5.0, 5.0},
    {3.0, 3.0, 3.0},
    {0.0, 0.0, 0.0}
  };

  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "case3_velocity_effort";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);
  SetUpAndActivateTrajectoryController(executor, params);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.effort = input_efforts_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();

}

//=============================================================================
// TEST CASE 4: Position + Velocity + Acceleration + Effort (Quintic)
// Input: position, velocity, acceleration, effort -> Output: position + velocity + acceleration + effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, case4_position_velocity_acceleration_effort)
{
  // Define consistent trajectory waypoints with 5 points and accelerations
  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.3, 0.3, 0.3},
    {1.0, 1.0, 1.0},
    {1.5, 1.5, 1.5},
    {2.0, 2.0, 2.0}
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {1.5, 1.5, 1.5},
    {2.0, 2.0, 2.0},
    {1.0, 1.0, 1.0},
    {0.0, 0.0, 0.0}
  };
  std::vector<std::vector<double>> input_accelerations_ = {
    {0.0, 0.0, 0.0},
    {3.0, 3.0, 3.0},
    {0.0, 0.0, 0.0},
    {-2.0, -2.0, -2.0},
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {2.0, 2.0, 2.0},
    {5.0, 5.0, 5.0},
    {3.0, 3.0, 3.0},
    {0.0, 0.0, 0.0}
  };

  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "case4_position_velocity_acceleration_effort";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);
  SetUpAndActivateTrajectoryController(executor, params);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.accelerations = input_accelerations_[i];
    point.effort = input_efforts_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  executor.cancel();

}


//=============================================================================
// TEST CASE 5: Position Only (No effort)
// Input: position only -> Output: position + velocity + acceleration + effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, case5_position_only)
{
  // Simple position-only trajectory
  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.5, 0.5, 0.5},
    {1.0, 1.0, 1.0},
    {1.5, 1.5, 1.5},
    {2.0, 2.0, 2.0}
  };
  input_velocities_ = {};  // No velocities
  input_efforts_ = {};     // No efforts

  command_interface_types_ = {"position"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "case5_position_only";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  SetUpAndActivateTrajectoryController(executor);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  EXPECT_NEAR(2.0, logged_data_.back().command_positions[0], EPS);
  executor.cancel();
}

//=============================================================================
// TEST CASE 6: Velocity Only
// Input: velocity only -> Output: position + velocity + acceleration + effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, case6_velocity_only)
{
  // Velocity-only trajectory (position integrated from velocity)
  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {0.5, 0.5, 0.5},
    {1.25, 1.25, 1.25},
    {1.75, 1.75, 1.75},
    {2.0, 2.0, 2.0}
  };
  input_velocities_ = {
    {1.0, 1.0, 1.0},
    {1.5, 1.5, 1.5},
    {1.0, 1.0, 1.0},
    {0.5, 0.5, 0.5},
    {0.0, 0.0, 0.0}
  };
  input_efforts_ = {};  // No efforts

  command_interface_types_ = {"position", "velocity"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "case6_velocity_only";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);
  SetUpAndActivateTrajectoryController(executor, params);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  EXPECT_NEAR(2.0, logged_data_.back().command_positions[0], EPS);
  executor.cancel();
}

//=============================================================================
// TEST CASE 7: Trapezoidal Velocity Profile
// Input: position with trapezoidal velocity profile -> Output: position + velocity + acceleration + effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, case7_trapezoidal_profile)
{
  // Trapezoidal velocity profile: accelerate, constant velocity, decelerate
  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},       // Start
    {0.25, 0.25, 0.25},    // End of acceleration
    {1.0, 1.0, 1.0},       // Constant velocity
    {1.75, 1.75, 1.75},    // Start of deceleration
    {2.0, 2.0, 2.0}        // End
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},       // Start at rest
    {1.0, 1.0, 1.0},       // Accelerated to max velocity
    {1.0, 1.0, 1.0},       // Constant velocity
    {1.0, 1.0, 1.0},       // Still at max velocity
    {0.0, 0.0, 0.0}        // Stop
  };
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0},
    {0.0, 0.0, 0.0},
    {-1.0, -1.0, -1.0},
    {0.0, 0.0, 0.0}
  };

  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "case7_trapezoidal_profile";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);
  SetUpAndActivateTrajectoryController(executor, params);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.effort = input_efforts_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  EXPECT_NEAR(2.0, logged_data_.back().command_positions[0], EPS);
  executor.cancel();
}

//=============================================================================
// TEST CASE 8: Zero Velocity Hold (Dwell)
// Input: position with zero velocities for hold/dwell behavior -> Output: position + velocity + acceleration + effort
//=============================================================================
TEST_F(TrajectoryInterpolationTest, case8_zero_velocity_hold)
{
  // Test hold/dwell behavior with zero velocities
  input_times_ = {0.0, 0.5, 1.0, 1.5, 2.0};
  input_positions_ = {
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 1.0},     // Move to position
    {1.0, 1.0, 1.0},     // Hold at position
    {1.0, 1.0, 1.0},     // Continue holding
    {2.0, 2.0, 2.0}      // Move to final position
  };
  input_velocities_ = {
    {0.0, 0.0, 0.0},
    {0.0, 0.0, 0.0},     // Stop
    {0.0, 0.0, 0.0},     // Hold (zero velocity)
    {0.0, 0.0, 0.0},     // Hold (zero velocity)
    {0.0, 0.0, 0.0}      // Stop
  };
  input_efforts_ = {
    {0.0, 0.0, 0.0},
    {2.0, 2.0, 2.0},
    {2.0, 2.0, 2.0},     // Holding effort
    {2.0, 2.0, 2.0},     // Holding effort
    {0.0, 0.0, 0.0}
  };

  command_interface_types_ = {"position", "velocity", "effort"};
  state_interface_types_ = {"position", "velocity", "effort"};
  test_config_name_ = "case8_zero_velocity_hold";
  interpolation_method_ = "linear";

  rclcpp::executors::MultiThreadedExecutor executor;
  std::vector<rclcpp::Parameter> params;
  params.emplace_back("open_loop_control", true);
  SetUpAndActivateTrajectoryController(executor, params);

  trajectory_msgs::msg::JointTrajectory traj_msg;
  traj_msg.joint_names = joint_names_;
  traj_msg.header.stamp = rclcpp::Time(0, 0);

  for (size_t i = 0; i < input_positions_.size(); ++i)
  {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = input_positions_[i];
    point.velocities = input_velocities_[i];
    point.effort = input_efforts_[i];
    point.time_from_start = rclcpp::Duration::from_seconds(input_times_[i]);
    traj_msg.points.push_back(point);
  }

  trajectory_publisher_->publish(traj_msg);
  traj_controller_->wait_for_trajectory(executor);

  print_log_header();
  print_input_trajectory();
  run_trajectory_with_logging(executor, 2.2, NUM_SAMPLES);
  print_sampled_trajectory();
  print_log_footer();

  EXPECT_NEAR(2.0, logged_data_.back().command_positions[0], EPS);
  executor.cancel();
}
