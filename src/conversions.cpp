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

[[nodiscard]] ::drake::trajectories::PiecewisePolynomial<double>
getPiecewisePolynomial(const ::robot_trajectory::RobotTrajectory& robot_trajectory,
                       const moveit::core::JointModelGroup* group)
{
  std::vector<double> breaks;
  breaks.reserve(robot_trajectory.getWayPointCount());
  std::vector<Eigen::MatrixXd> samples;
  samples.reserve(robot_trajectory.getWayPointCount());

  // Create samples & breaks
  for (std::size_t i = 0; i < robot_trajectory.getWayPointCount(); ++i)
  {
    const auto& state = robot_trajectory.getWayPoint(i);
    Eigen::VectorXd position(state.getVariableCount());
    state.copyJointGroupPositions(group, position);
    samples.emplace_back(position);
    breaks.emplace_back(robot_trajectory.getWayPointDurationFromStart(i));
  }

  // Create a piecewise polynomial trajectory
  return ::drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
}

void getRobotTrajectory(const ::drake::trajectories::Trajectory<double>& drake_trajectory, const double delta_t,
                        std::shared_ptr<::robot_trajectory::RobotTrajectory>& moveit_trajectory)
{
  // Get the start and end times of the piecewise polynomial
  double t_prev = 0.0;
  const auto num_pts = static_cast<size_t>(std::ceil(drake_trajectory.end_time() / delta_t) + 1);

  for (unsigned int i = 0; i < num_pts; ++i)
  {
    const auto t_scale = static_cast<double>(i) / static_cast<double>(num_pts - 1);
    const auto t = std::min(t_scale, 1.0) * drake_trajectory.end_time();
    const auto pos_val = drake_trajectory.value(t);
    const auto vel_val = drake_trajectory.EvalDerivative(t);
    const auto waypoint = std::make_shared<moveit::core::RobotState>(moveit_trajectory->getRobotModel());
    const auto active_joints = moveit_trajectory->getRobotModel()->getActiveJointModels();
    for (size_t joint_index = 0; joint_index < active_joints.size(); ++joint_index)
    {
      waypoint->setJointPositions(active_joints[joint_index], &pos_val(joint_index));
      waypoint->setJointVelocities(active_joints[joint_index], &vel_val(joint_index));
    }

    moveit_trajectory->addSuffixWayPoint(waypoint, t - t_prev);
    t_prev = t;
  }
}
}  // namespace moveit::drake
