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

#include <ros/ros.h>
#include <rpg_mpc/mpc_wrapper.h>

#include "quadrotor_common/parameter_helper.h"

namespace rpg_mpc {

template <typename T>
class MpcParams {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MpcParams()
      : changed_(false),
        print_info_(false),
        state_cost_exponential_(0.0),
        input_cost_exponential_(0.0),
        max_bodyrate_xy_(0.0),
        max_bodyrate_z_(0.0),
        min_thrust_(0.0),
        max_thrust_(0.0),
        p_B_C_(Eigen::Matrix<T, 3, 1>::Zero()),
        q_B_C_(Eigen::Quaternion<T>(1.0, 0.0, 0.0, 0.0)),
        Q_(Eigen::Matrix<T, kCostSize, kCostSize>::Zero()),
        R_(Eigen::Matrix<T, kInputSize, kInputSize>::Zero()) {}

  ~MpcParams() {}

  bool loadParameters(ros::NodeHandle& pnh) {
#define GET_PARAM(name) \
  if (!quadrotor_common::getParam(#name, name, pnh)) return false

#define GET_PARAM_(name) \
  if (!quadrotor_common::getParam(#name, name##_, pnh)) return false

    // Read state costs.
    T Q_pos_xy, Q_pos_z, Q_attitude, Q_velocity, Q_perception;
    GET_PARAM(Q_pos_xy);
    GET_PARAM(Q_pos_z);
    GET_PARAM(Q_attitude);
    GET_PARAM(Q_velocity);
    quadrotor_common::getParam("Q_perception", Q_perception, (T)0.0, pnh);

    // Check whether all state costs are positive.
    if (Q_pos_xy <= 0.0 || Q_pos_z <= 0.0 || Q_attitude <= 0.0 ||
        Q_velocity <= 0.0 ||
        Q_perception < 0.0)  // Perception cost can be zero to deactivate.
    {
      ROS_ERROR("MPC: State cost Q has negative enries!");
      return false;
    }

    // Read input costs.
    T R_thrust, R_pitchroll, R_yaw;
    GET_PARAM(R_thrust);
    GET_PARAM(R_pitchroll);
    GET_PARAM(R_yaw);

    // Check whether all input costs are positive.
    if (R_thrust <= 0.0 || R_pitchroll <= 0.0 || R_yaw <= 0.0) {
      ROS_ERROR("MPC: Input cost R has negative enries!");
      return false;
    }

    // Set state and input cost matrices.
    Q_ = (Eigen::Matrix<T, kCostSize, 1>() << Q_pos_xy, Q_pos_xy, Q_pos_z,
          Q_attitude, Q_attitude, Q_attitude, Q_attitude, Q_velocity,
          Q_velocity, Q_velocity, Q_perception, Q_perception)
             .finished()
             .asDiagonal();
    R_ = (Eigen::Matrix<T, kInputSize, 1>() << R_thrust, R_pitchroll,
          R_pitchroll, R_yaw)
             .finished()
             .asDiagonal();

    // Read cost scaling values
    quadrotor_common::getParam("state_cost_exponential",
                               state_cost_exponential_, (T)0.0, pnh);
    quadrotor_common::getParam("input_cost_exponential",
                               input_cost_exponential_, (T)0.0, pnh);

    // Read input limits.
    GET_PARAM_(max_bodyrate_xy);
    GET_PARAM_(max_bodyrate_z);
    GET_PARAM_(min_thrust);
    GET_PARAM_(max_thrust);

    // Check whether all input limits are positive.
    if (max_bodyrate_xy_ <= 0.0 || max_bodyrate_z_ <= 0.0 ||
        min_thrust_ <= 0.0 || max_thrust_ <= 0.0) {
      ROS_ERROR("MPC: All limits must be positive non-zero values!");
      return false;
    }

    // Optional parameters
    std::vector<T> p_B_C(3), q_B_C(4);
    if (!pnh.getParam("p_B_C", p_B_C)) {
      ROS_WARN("MPC: Camera extrinsic translation is not set.");
    } else {
      p_B_C_ = Eigen::Matrix<T, 3, 1>(p_B_C[0], p_B_C[1], p_B_C[2]);
    }
    if (!pnh.getParam("q_B_C", q_B_C)) {
      ROS_WARN("MPC: Camera extrinsic rotation is not set.");
    } else {
      q_B_C_ = Eigen::Quaternion<T>(q_B_C[0], q_B_C[1], q_B_C[2], q_B_C[3]);
    }

    quadrotor_common::getParam("print_info", print_info_, false, pnh);
    if (print_info_) ROS_INFO("MPC: Informative printing enabled.");

    changed_ = true;

#undef GET_PARAM
#undef GET_PARAM_OPT
#undef GET_PARAM_
#undef GET_PARAM_OPT_

    return true;
  }

  bool changed_;

  bool print_info_;

  T state_cost_exponential_;
  T input_cost_exponential_;

  T max_bodyrate_xy_;
  T max_bodyrate_z_;
  T min_thrust_;
  T max_thrust_;

  Eigen::Matrix<T, 3, 1> p_B_C_;
  Eigen::Quaternion<T> q_B_C_;

  Eigen::Matrix<T, kCostSize, kCostSize> Q_;
  Eigen::Matrix<T, kInputSize, kInputSize> R_;
};  // namespace rpg_mpc

}  // namespace rpg_mpc