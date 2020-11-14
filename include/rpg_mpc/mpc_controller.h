/*    rpg_quadrotor_mpc
 *    A model predictive control implementation for quadrotors.
 *    Copyright (C) 2017-2018 Philipp Foehn,
 *    Robotics and Perception Group, University of Zurich
 *
 *    Intended to be used with rpg_quadrotor_control and rpg_quadrotor_common.
 *    https://github.com/uzh-rpg/rpg_quadrotor_control
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <thread>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <quadrotor_common/control_command.h>
#include <quadrotor_common/quad_state_estimate.h>
#include <quadrotor_common/trajectory.h>
#include <quadrotor_common/trajectory_point.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <Eigen/Eigen>

#include "rpg_mpc/mpc_params.h"
#include "rpg_mpc/mpc_wrapper.h"

namespace rpg_mpc {

enum STATE {
  kPosX = 0,
  kPosY = 1,
  kPosZ = 2,
  kOriW = 3,
  kOriX = 4,
  kOriY = 5,
  kOriZ = 6,
  kVelX = 7,
  kVelY = 8,
  kVelZ = 9
};

enum INPUT { kThrust = 0, kRateX = 1, kRateY = 2, kRateZ = 3 };

template <typename T>
class MpcController {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  static_assert(
      kStateSize == 10,
      "MpcController: Wrong model size. Number of states does not match.");
  static_assert(
      kInputSize == 4,
      "MpcController: Wrong model size. Number of inputs does not match.");

  MpcController(const ros::NodeHandle& nh, const ros::NodeHandle& pnh,
                const std::string& topic = "mpc/trajectory_predicted");

  MpcController() : MpcController(ros::NodeHandle(), ros::NodeHandle("~")) {}

  quadrotor_common::ControlCommand off();

  quadrotor_common::ControlCommand run(
      const quadrotor_common::QuadStateEstimate& state_estimate,
      const quadrotor_common::Trajectory& reference_trajectory,
      const MpcParams<T>& params);

  void run(const quadrotor_common::QuadStateEstimate& state_estimate,
           const quadrotor_common::Trajectory& reference_trajectory,
           const MpcParams<T>& params, const bool solve_from_scratch, double* optimized_states,
           double* optimized_inputs);

 private:
  // Internal helper functions.

  void pointOfInterestCallback(
      const geometry_msgs::PointStamped::ConstPtr& msg);

  void offCallback(const std_msgs::Empty::ConstPtr& msg);

  bool setStateEstimate(
      const quadrotor_common::QuadStateEstimate& state_estimate);

  bool setReference(const quadrotor_common::Trajectory& reference_trajectory);

  quadrotor_common::ControlCommand updateControlCommand(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, 1>> input,
      ros::Time& time);

  bool publishPrediction(
      const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples + 1>> states,
      const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples>> inputs,
      ros::Time& time);

  void preparationThread();

  bool setNewParams(MpcParams<T>& params);

  // Handles
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // Subscribers and publisher.
  ros::Subscriber sub_point_of_interest_;
  ros::Subscriber sub_autopilot_off_;
  ros::Publisher pub_predicted_trajectory_;

  // Parameters
  MpcParams<T> params_;

  // MPC
  MpcWrapper<T> mpc_wrapper_;

  // Preparation Thread
  std::thread preparation_thread_;

  // Variables
  T timing_feedback_, timing_preparation_;
  bool solve_from_scratch_;
  Eigen::Matrix<T, kStateSize, 1> est_state_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> reference_states_;
  Eigen::Matrix<T, kInputSize, kSamples + 1> reference_inputs_;
  Eigen::Matrix<T, kStateSize, kSamples + 1> predicted_states_;
  Eigen::Matrix<T, kInputSize, kSamples> predicted_inputs_;
  Eigen::Matrix<T, 3, 1> point_of_interest_;
};

}  // namespace rpg_mpc
