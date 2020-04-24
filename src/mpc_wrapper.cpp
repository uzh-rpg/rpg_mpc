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


#include "rpg_mpc/mpc_wrapper.h"


namespace rpg_mpc {

// Default Constructor.
template <typename T>
MpcWrapper<T>::MpcWrapper()
{
  // Clear solver memory.
  memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
  memset(&acadoVariables, 0, sizeof( acadoVariables ));
  
  // Initialize the solver.
  acado_initializeSolver();

  // Initialize the states and controls.
  const Eigen::Matrix<T, kStateSize, 1> hover_state =
    (Eigen::Matrix<T, kStateSize, 1>() << 0.0, 0.0, 0.0,
                                          1.0, 0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0).finished();

  // Initialize states x and xN and input u.
  acado_initial_state_ = hover_state.template cast<float>();

  acado_states_ = hover_state.replicate(1, kSamples+1).template cast<float>();

  acado_inputs_ = kHoverInput_.replicate(1, kSamples).template cast<float>();

  // Initialize references y and yN.
  acado_reference_states_.block(0, 0, kStateSize, kSamples) =
    hover_state.replicate(1, kSamples).template cast<float>();

  acado_reference_states_.block(kStateSize, 0, kCostSize-kStateSize, kSamples) =
    Eigen::Matrix<float, kCostSize-kStateSize, kSamples>::Zero();

  acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
    kHoverInput_.replicate(1, kSamples);

  acado_reference_end_state_.segment(0, kStateSize) =
    hover_state.template cast<float>();

  acado_reference_end_state_.segment(kStateSize, kCostSize-kStateSize) =
    Eigen::Matrix<float, kCostSize-kStateSize, 1>::Zero();

  // Initialize Cost matrix W and WN.
  if(!(acado_W_.trace()>0.0))
  {
    acado_W_ = W_.replicate(1, kSamples).template cast<float>();
    acado_W_end_ = WN_.template cast<float>();
  }

  // Initialize online data.
  Eigen::Matrix<T, 3, 1> p_B_C(0, 0, 0);
  Eigen::Quaternion<T> q_B_C(1, 0, 0, 0);
  Eigen::Matrix<T, 3, 1> point_of_interest(0, 0, -1000);

  setCameraParameters(p_B_C, q_B_C);
  setPointOfInterest(point_of_interest);

  // Initialize solver.
  acado_initializeNodesByForwardSimulation();
  acado_preparationStep();
  acado_is_prepared_ = true;
}

// Constructor with cost matrices as arguments.
template <typename T>
MpcWrapper<T>::MpcWrapper(
  const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R)
{
  setCosts(Q, R);
  MpcWrapper();
}

// Set cost matrices with optional scaling.
template <typename T>
bool MpcWrapper<T>::setCosts(
  const Eigen::Ref<const Eigen::Matrix<T, kCostSize, kCostSize>> Q,
  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kInputSize>> R,
  const T state_cost_scaling, const T input_cost_scaling)
{
  if(state_cost_scaling < 0.0 || input_cost_scaling < 0.0 )
  {
    ROS_ERROR("MPC: Cost scaling is wrong, must be non-negative!");
    return false;
  }
  W_.block(0, 0, kCostSize, kCostSize) = Q;
  W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R;
  WN_ = W_.block(0, 0, kCostSize, kCostSize);

  float state_scale{1.0};
  float input_scale{1.0};
  for(int i=0; i<kSamples; i++)
  { 
    state_scale = exp(- float(i)/float(kSamples)
      * float(state_cost_scaling));
    input_scale = exp(- float(i)/float(kSamples)
      * float(input_cost_scaling));
    acado_W_.block(0, i*kRefSize, kCostSize, kCostSize) =
      W_.block(0, 0, kCostSize, kCostSize).template cast<float>()
      * state_scale;
    acado_W_.block(kCostSize, i*kRefSize+kCostSize, kInputSize, kInputSize) =
      W_.block(kCostSize, kCostSize, kInputSize, kInputSize
        ).template cast<float>() * input_scale;
  } 
  acado_W_end_ = WN_.template cast<float>() * state_scale;

  return true;
}

// Set the input limits.
template <typename T>
bool MpcWrapper<T>::setLimits(T min_thrust, T max_thrust,
    T max_rollpitchrate, T max_yawrate)
{
  if(min_thrust <= 0.0 || min_thrust > max_thrust)
  {
    ROS_ERROR("MPC: Minimal thrust is not set properly, not changed.");
    return false;
  }

  if(max_thrust <= 0.0 || min_thrust > max_thrust)
  {
    ROS_ERROR("MPC: Maximal thrust is not set properly, not changed.");
    return false;
  }

  if(max_rollpitchrate <= 0.0)
  {
    ROS_ERROR("MPC: Maximal xy-rate is not set properly, not changed.");
    return false;
  }

  if(max_yawrate <= 0.0)
  {
    ROS_ERROR("MPC: Maximal yaw-rate is not set properly, not changed.");
    return false;
  }

  // Set input boundaries.
  Eigen::Matrix<T, 4, 1> lower_bounds = Eigen::Matrix<T, 4, 1>::Zero();
  Eigen::Matrix<T, 4, 1> upper_bounds = Eigen::Matrix<T, 4, 1>::Zero();
  lower_bounds << min_thrust,
    -max_rollpitchrate, -max_rollpitchrate, -max_yawrate;
  upper_bounds << max_thrust,
    max_rollpitchrate, max_rollpitchrate, max_yawrate;

  acado_lower_bounds_ =
    lower_bounds.replicate(1, kSamples).template cast<float>();

  acado_upper_bounds_ =
    upper_bounds.replicate(1, kSamples).template cast<float>();
  return true;
}

// Set camera extrinsics.
template <typename T>
bool MpcWrapper<T>::setCameraParameters(
  const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& p_B_C,
  Eigen::Quaternion<T>& q_B_C)
{
  acado_online_data_.block(3, 0, 3, ACADO_N+1)
    = p_B_C.replicate(1, ACADO_N+1).template cast<float>();

  Eigen::Matrix<T, 4, 1> q_B_C_mat(
    q_B_C.w(), q_B_C.x(), q_B_C.y(), q_B_C.z());
  acado_online_data_.block(6, 0, 4, ACADO_N+1)
    = q_B_C_mat.replicate(1, ACADO_N+1).template cast<float>();

  return true;
}

// Set the point of interest. Perception cost should be non-zero.
template <typename T>
bool MpcWrapper<T>::setPointOfInterest(
  const Eigen::Ref<const Eigen::Matrix<T, 3, 1>>& position)
{
  acado_online_data_.block(0, 0, 3, ACADO_N+1)
    = position.replicate(1, ACADO_N+1).template cast<float>();
  return true;
}

// Set a reference pose.
template <typename T>
bool MpcWrapper<T>::setReferencePose(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
  acado_reference_states_.block(0, 0, kStateSize, kSamples) =
    state.replicate(1, kSamples).template cast<float>();

  acado_reference_states_.block(kStateSize, 0, kCostSize-kStateSize, kSamples) =
    Eigen::Matrix<float, kCostSize-kStateSize, kSamples>::Zero();

  acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
    kHoverInput_.replicate(1, kSamples);

  acado_reference_end_state_.segment(0, kStateSize) =
    state.template cast<float>();

  acado_reference_end_state_.segment(kStateSize, kCostSize-kStateSize) =
    Eigen::Matrix<float, kCostSize-kStateSize, 1>::Zero();

  acado_initializeNodesByForwardSimulation();
  return true;
}

// Set a reference trajectory.
template <typename T>
bool MpcWrapper<T>::setTrajectory(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, kSamples+1>> states,
  const Eigen::Ref<const Eigen::Matrix<T, kInputSize, kSamples+1>> inputs)
{
  Eigen::Map<Eigen::Matrix<float, kRefSize, kSamples, Eigen::ColMajor>>
    y(const_cast<float*>(acadoVariables.y));

  acado_reference_states_.block(0, 0, kStateSize, kSamples) =
    states.block(0, 0, kStateSize, kSamples).template cast<float>();

  acado_reference_states_.block(kStateSize, 0, kCostSize-kStateSize, kSamples) =
    Eigen::Matrix<float, kCostSize-kStateSize, kSamples>::Zero();

  acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
    inputs.block(0, 0, kInputSize, kSamples).template cast<float>();

  acado_reference_end_state_.segment(0, kStateSize) =
    states.col(kSamples).template cast<float>();
  acado_reference_end_state_.segment(kStateSize, kCostSize-kStateSize) =
    Eigen::Matrix<float, kCostSize-kStateSize, 1>::Zero();

  return true;
}

// Reset states and inputs and calculate new solution.
template <typename T>
bool MpcWrapper<T>::solve(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state)
{
  acado_states_ = state.replicate(1, kSamples+1).template cast<float>();

  acado_inputs_ = kHoverInput_.replicate(1, kSamples);
  
  prepare();

  return update(state);
}


// Calculate new solution from last known solution.
template <typename T>
bool MpcWrapper<T>::update(
  const Eigen::Ref<const Eigen::Matrix<T, kStateSize, 1>> state,
  bool do_preparation)
{
  if(!acado_is_prepared_)
  {
    ROS_WARN("MPC: Solver was triggered without preparation, abort!");
    return false;
  }

  // Check if estimated and reference quaternion live in the same hemisphere.
  acado_initial_state_ = state.template cast<float>();
  if(acado_initial_state_.segment(3,4).dot(
    Eigen::Vector4f(acado_reference_states_.block(3,0,4,1)))<(T)0.0)
  {
    acado_initial_state_.segment(3,4) = -acado_initial_state_.segment(3,4);
  }

  // Perform feedback step and reset preparation check.
  acado_feedbackStep();
  acado_is_prepared_ = false;

  // Prepare if the solver if wanted
  if(do_preparation)
  {
    acado_preparationStep();
    acado_is_prepared_ = true;
  }

  return true;
}

// Prepare the solver.
// Must be triggered between iterations if not done in the update function.
template <typename T>
bool MpcWrapper<T>::prepare()
{
  acado_preparationStep();
  acado_is_prepared_ = true;

  return true;
}

// Get a specific state.
template <typename T>
void MpcWrapper<T>::getState(const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kStateSize, 1>> return_state)
{
  return_state = acado_states_.col(node_index).cast<T>();
}

// Get all states.
template <typename T>
void MpcWrapper<T>::getStates(
    Eigen::Ref<Eigen::Matrix<T, kStateSize, kSamples+1>> return_states)
{
  return_states = acado_states_.cast<T>();
}

// Get a specific input.
template <typename T>
void MpcWrapper<T>::getInput(const int node_index,
    Eigen::Ref<Eigen::Matrix<T, kInputSize, 1>> return_input)
{
  return_input = acado_inputs_.col(node_index).cast<T>();
}

// Get all inputs.
template <typename T>
void MpcWrapper<T>::getInputs(
    Eigen::Ref<Eigen::Matrix<T, kInputSize, kSamples>> return_inputs)
{
  return_inputs = acado_inputs_.cast<T>();
}


template class MpcWrapper<float>;
template class MpcWrapper<double>;

} // namespace rpg_mpc

