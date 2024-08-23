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
   Desc: TODO
*/

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>

#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/trajectories/trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>

namespace moveit::drake
{
/**
 * @brief Create a Piecewise Polynomial from a moveit trajectory (see
 * https://drake.mit.edu/doxygen_cxx/classdrake_1_1trajectories_1_1_piecewise_polynomial.html)
 *
 * @param robot_trajectory MoveIt trajectory to be translated
 * @param group Joint group for which a piecewise polynomial is created
 * @return ::drake::trajectories::PiecewisePolynomial<double>
 */
[[nodiscard]] ::drake::trajectories::PiecewisePolynomial<double>
getPiecewisePolynomial(const ::robot_trajectory::RobotTrajectory& robot_trajectory,
                       const moveit::core::JointModelGroup* group);

/**
 * @brief Create a moveit trajectory from a piecewise polynomial. Assumes that the piecewise polynomial describes a
 * joint trajectory for every active joint of the given trajectory.
 *
 * @param drake_trajectory Drake trajectory
 * @param delta_t Time step size
 * @param moveit_trajectory MoveIt trajectory to be populated based on the piecewise polynomial
 */
void getRobotTrajectory(const ::drake::trajectories::Trajectory<double>& drake_trajectory, const double delta_t,
                        std::shared_ptr<::robot_trajectory::RobotTrajectory>& moveit_trajectory);

/**
 * @brief Get a joint positions vector for a MultibodyPlant from a MoveIt robot state
 *
 * @param moveit_state MoveIt Robot state
 * @param group_name Name of the joint group
 * @param plant Drake Multibody Plant
 * @return Vector with drake joint positions
 */
[[nodiscard]] Eigen::VectorXd getJointPositions(const moveit::core::RobotState& moveit_state,
                                                const std::string& group_name,
                                                const ::drake::multibody::MultibodyPlant<double>& plant);

/**
 * @brief Get a joint velocities vector for a MultibodyPlant from a MoveIt robot state
 *
 * @param moveit_state MoveIt Robot state
 * @param group_name Name of the joint group
 * @param plant Drake Multibody Plant
 * @return Vector with drake joint velocities
 */
[[nodiscard]] Eigen::VectorXd getJointVelocities(const moveit::core::RobotState& moveit_state,
                                                 const std::string& group_name,
                                                 const ::drake::multibody::MultibodyPlant<double>& plant);
}  // namespace moveit::drake
