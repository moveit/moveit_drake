/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2024, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sebastian Jahr
 */

#include <moveit/drake/conversions.hpp>
namespace moveit::drake
{
using ::drake::geometry::SceneGraph;
using ::drake::multibody::AddMultibodyPlantSceneGraph;
using ::drake::multibody::MultibodyPlant;
using ::drake::multibody::Parser;
using ::drake::systems::DiagramBuilder;

[[nodiscard]] Eigen::VectorXd getJointPositionVector(const moveit::core::RobotState& moveit_state,
                                                     const std::string& group_name, const MultibodyPlant<double>& plant)
{
  const auto& joint_model_group = moveit_state.getRobotModel()->getJointModelGroup(group_name);
  assert(plant.num_positions() >= joint_model_group->getActiveJointModels().size());
  Eigen::VectorXd joint_positions = Eigen::VectorXd::Zero(plant.num_positions());

  for (const auto& joint_model : joint_model_group->getActiveJointModels())
  {
    const auto& joint_name = joint_model->getName();
    const auto& joint_index = plant.GetJointByName(joint_name).ordinal();
    joint_positions(joint_index) = moveit_state.getVariablePosition(joint_name);
  }
  std::cout << "Joint positions: " << joint_positions.transpose() << std::endl;
  return joint_positions;
}

[[nodiscard]] Eigen::VectorXd getJointVelocityVector(const moveit::core::RobotState& moveit_state,
                                                     const std::string& group_name, const MultibodyPlant<double>& plant)
{
  const auto& joint_model_group = moveit_state.getRobotModel()->getJointModelGroup(group_name);
  assert(plant.num_velocities() >= joint_model_group->getActiveJointModels().size());
  Eigen::VectorXd joint_velocities = Eigen::VectorXd::Zero(plant.num_velocities());
  for (const auto& joint_model : joint_model_group->getActiveJointModels())
  {
    const auto& joint_name = joint_model->getName();
    const auto& joint_index = plant.GetJointByName(joint_name).ordinal();
    joint_velocities(joint_index) = moveit_state.getVariableVelocity(joint_name);
  }
  std::cout << "Joint velocities: " << joint_velocities.transpose() << std::endl;
  return joint_velocities;
}

void getVelocityBounds(const moveit::core::JointModelGroup* joint_model_group, const MultibodyPlant<double>& plant,
                       Eigen::VectorXd& lower_velocity_bounds, Eigen::VectorXd& upper_velocity_bounds)
{
  assert(plant.num_velocities() >= joint_model_group->getActiveJointModels().size());
  lower_velocity_bounds.resize(plant.num_velocities());
  upper_velocity_bounds.resize(plant.num_velocities());
  for (const auto& joint_model : joint_model_group->getActiveJointModels())
  {
    const moveit::core::VariableBounds& bounds = joint_model->getVariableBounds()[0];  // Assume single DoF joints
    const auto& joint_name = joint_model->getName();
    const auto& joint_index = plant.GetJointByName(joint_name).ordinal();

    if (bounds.velocity_bounded_)
    {
      lower_velocity_bounds(joint_index) = bounds.min_velocity_;
      upper_velocity_bounds(joint_index) = bounds.max_velocity_;
    }
    else
    {
      lower_velocity_bounds(joint_index) = -std::numeric_limits<double>::max();
      upper_velocity_bounds(joint_index) = std::numeric_limits<double>::max();
    }
  }
}

void getAccelerationBounds(const moveit::core::JointModelGroup* joint_model_group, const MultibodyPlant<double>& plant,
                           Eigen::VectorXd& lower_acceleration_bounds, Eigen::VectorXd& upper_acceleration_bounds)
{
  assert(plant.num_accelerations() >= joint_model_group->getActiveJointModels().size());
  lower_acceleration_bounds.resize(plant.num_velocities());
  upper_acceleration_bounds.resize(plant.num_velocities());
  for (const auto& joint_model : joint_model_group->getActiveJointModels())
  {
    const moveit::core::VariableBounds& bounds = joint_model->getVariableBounds()[0];  // Assume single DoF joints
    const auto& joint_name = joint_model->getName();
    const auto& joint_index = plant.GetJointByName(joint_name).ordinal();

    if (bounds.velocity_bounded_)
    {
      lower_acceleration_bounds(joint_index) = bounds.min_acceleration_;
      upper_acceleration_bounds(joint_index) = bounds.max_acceleration_;
    }
    else
    {
      lower_acceleration_bounds(joint_index) = -std::numeric_limits<double>::max();
      upper_acceleration_bounds(joint_index) = std::numeric_limits<double>::max();
    }
  }
}

[[nodiscard]] ::drake::trajectories::PiecewisePolynomial<double>
getPiecewisePolynomial(const ::robot_trajectory::RobotTrajectory& robot_trajectory,
                       const moveit::core::JointModelGroup* group, const MultibodyPlant<double>& plant)
{
  std::vector<double> breaks;
  breaks.reserve(robot_trajectory.getWayPointCount());
  std::vector<Eigen::MatrixXd> samples;
  samples.reserve(robot_trajectory.getWayPointCount());

  // Create samples & breaks
  for (std::size_t i = 0; i < robot_trajectory.getWayPointCount(); ++i)
  {
    const auto& state = robot_trajectory.getWayPoint(i);
    samples.emplace_back(getJointPositionVector(state, group->getName(), plant));
    breaks.emplace_back(robot_trajectory.getWayPointDurationFromStart(i));
  }

  // Create a piecewise polynomial trajectory
  return ::drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
}

void getRobotTrajectory(const ::drake::trajectories::Trajectory<double>& drake_trajectory, const double delta_t,
                        const MultibodyPlant<double>& plant,
                        std::shared_ptr<::robot_trajectory::RobotTrajectory>& moveit_trajectory)
{
  // Reset output trajectory
  moveit_trajectory->clear();

  // Get the start and end times of the piecewise polynomial
  double t_prev = 0.0;
  const auto num_pts = static_cast<size_t>(std::ceil(drake_trajectory.end_time() / delta_t) + 1);

  const auto active_joints = moveit_trajectory->getGroup()->getActiveJointModels();
  for (unsigned int i = 0; i < num_pts; ++i)
  {
    const auto t_scale = static_cast<double>(i) / static_cast<double>(num_pts - 1);
    const auto t = std::min(t_scale, 1.0) * drake_trajectory.end_time();
    const auto pos_val = drake_trajectory.value(t);
    std::cout << "pos_val size: " << pos_val.size() << std::endl;
    const auto vel_val = drake_trajectory.EvalDerivative(t);
    const auto waypoint = std::make_shared<moveit::core::RobotState>(moveit_trajectory->getRobotModel());
    for (const auto& joint_model : active_joints)
    {
      const auto& joint_name = joint_model->getName();
      const auto& joint_index = plant.GetJointByName(joint_name).ordinal();
      waypoint->setJointPositions(joint_model, &pos_val(joint_index));
      waypoint->setJointVelocities(joint_model, &vel_val(joint_index));
    }

    moveit_trajectory->addSuffixWayPoint(waypoint, t - t_prev);
    t_prev = t;
  }
}
}  // namespace moveit::drake
