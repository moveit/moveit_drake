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

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>

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
 * @brief Get a joint positions vector for a MultibodyPlant from a MoveIt robot state
 *
 * @param moveit_state MoveIt Robot state
 * @param group_name Name of the joint group
 * @param plant Drake Multibody Plant
 * @return Vector with drake joint positions
 */
[[nodiscard]] Eigen::VectorXd getJointPositionVector(const moveit::core::RobotState& moveit_state,
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
[[nodiscard]] Eigen::VectorXd getJointVelocityVector(const moveit::core::RobotState& moveit_state,
                                                     const std::string& group_name,
                                                     const ::drake::multibody::MultibodyPlant<double>& plant);

/**
 * @brief Copy position bounds from joint model group to Eigen vectors
 *
 * @param joint_model_group Joint model group to get position bounds from
 * @param plant Drake MultiBody Plant, used to get model information
 * @param lower_position_bounds Lower position bounds populated by this function
 * @param upper_position_bounds Upper position bounds populated by this function
 */
void getPositionBounds(const moveit::core::JointModelGroup* joint_model_group,
                       const ::drake::multibody::MultibodyPlant<double>& plant, Eigen::VectorXd& lower_position_bounds,
                       Eigen::VectorXd& upper_position_bounds);

/**
 * @brief Copy velocity bounds from joint model group to Eigen vectors
 *
 * @param joint_model_group Joint model group to get velocity bounds from
 * @param plant Drake MultiBody Plant, used to get model information
 * @param lower_velocity_bounds Lower velocity bounds populated by this function
 * @param upper_velocity_bounds Upper velocity bounds populated by this function
 */
void getVelocityBounds(const moveit::core::JointModelGroup* joint_model_group,
                       const ::drake::multibody::MultibodyPlant<double>& plant, Eigen::VectorXd& lower_velocity_bounds,
                       Eigen::VectorXd& upper_velocity_bounds);

/**
 * @brief Get the Acceleration Bounds object
 *
 * @param joint_model_group Joint model group to get acceleration bounds from
 * @param plant Drake model plant, used to get model information
 * @param lower_acceleration_bounds Lower acceleration bounds ppulated by this function
 * @param upper_acceleration_bounds Upper acceleration bounds populated by this function
 */
void getAccelerationBounds(const moveit::core::JointModelGroup* joint_model_group,
                           const ::drake::multibody::MultibodyPlant<double>& plant,
                           Eigen::VectorXd& lower_acceleration_bounds, Eigen::VectorXd& upper_acceleration_bounds);

/**
 * @brief Get the Jerk Bounds object
 *
 * @param joint_model_group Joint model group to get jerk bounds from
 * @param plant Drake model plant, used to get model information
 * @param lower_jerk_bounds Lower jerk bounds populated by this function
 * @param upper_jerk_bounds Upper jerk bounds populated by this function
 */
void getJerkBounds(const moveit::core::JointModelGroup* joint_model_group,
                   const ::drake::multibody::MultibodyPlant<double>& plant, Eigen::VectorXd& lower_jerk_bounds,
                   Eigen::VectorXd& upper_jerk_bounds);

/**
 * @brief Create a Piecewise Polynomial from a moveit trajectory (see
 * https://drake.mit.edu/doxygen_cxx/classdrake_1_1trajectories_1_1_piecewise_polynomial.html)
 *
 * @param robot_trajectory MoveIt trajectory to be translated
 * @param group Joint group for which a piecewise polynomial is created
 * @param plant Drake Multibody Plant, used to get model information
 * @return ::drake::trajectories::PiecewisePolynomial<double>
 */
[[nodiscard]] ::drake::trajectories::PiecewisePolynomial<double>
getPiecewisePolynomial(const ::robot_trajectory::RobotTrajectory& robot_trajectory,
                       const moveit::core::JointModelGroup* group,
                       const ::drake::multibody::MultibodyPlant<double>& plant);

/**
 * @brief Create a moveit trajectory from a piecewise polynomial. Assumes that the piecewise polynomial describes a
 * joint trajectory for every active joint of the given trajectory.
 *
 * @param drake_trajectory Drake trajectory
 * @param delta_t Time step size
 * @param plant Drake Multibody Plant, used to get model information
 * @param moveit_trajectory MoveIt trajectory to be populated based on the piecewise polynomial
 */
void getRobotTrajectory(const ::drake::trajectories::Trajectory<double>& drake_trajectory, const double delta_t,
                        const ::drake::multibody::MultibodyPlant<double>& plant,
                        std::shared_ptr<::robot_trajectory::RobotTrajectory>& moveit_trajectory);

/**
 * @brief Converts all STL file paths in a URDF string to OBJ file paths
 *
 * @param input Input robot description
 * @return std::string Robot description with all STL file paths replaced by OBJ file paths
 */
[[nodiscard]] std::string replaceSTLWithOBJ(const std::string& input);
}  // namespace moveit::drake
