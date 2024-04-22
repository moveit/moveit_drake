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

/* Author: Sebastian Jahr */

#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit/drake/diff_ik_plugin.hpp>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Drake
#include <drake/multibody/inverse_kinematics/differential_inverse_kinematics.h>

// register SRVKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(diff_ik_plugin::DiffIKPlugin, kinematics::KinematicsBase)

namespace diff_ik_plugin
{

using namespace drake::multibody;
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.kinematics.diff_ik_plugin");
}
}  // namespace

DiffIKPlugin::DiffIKPlugin()
{
}

bool DiffIKPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                              const std::string& group_name, const std::string& base_frame,
                              const std::vector<std::string>& tip_frames, double search_discretization)
{
  // TODO
  return true;
}

bool DiffIKPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  // TODO
  return true;
}

bool DiffIKPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                 std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                 const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, consistency_limits, solution, IKCallbackFn(),
                          error_code, options);
}

bool DiffIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                    double timeout, std::vector<double>& solution,
                                    moveit_msgs::msg::MoveItErrorCodes& error_code,
                                    const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool DiffIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                    double timeout, const std::vector<double>& consistency_limits,
                                    std::vector<double>& solution, moveit_msgs::msg::MoveItErrorCodes& error_code,
                                    const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool DiffIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                    double timeout, std::vector<double>& solution,
                                    const IKCallbackFn& solution_callback,
                                    moveit_msgs::msg::MoveItErrorCodes& error_code,
                                    const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool DiffIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                                    double timeout, const std::vector<double>& consistency_limits,
                                    std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                    moveit_msgs::msg::MoveItErrorCodes& error_code,
                                    const kinematics::KinematicsQueryOptions& options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::msg::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool DiffIKPlugin::searchPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
                                    const std::vector<double>& ik_seed_state, double /*timeout*/,
                                    const std::vector<double>& /*consistency_limits*/, std::vector<double>& solution,
                                    const IKCallbackFn& solution_callback,
                                    moveit_msgs::msg::MoveItErrorCodes& error_code,
                                    const kinematics::KinematicsQueryOptions& /*options*/,
                                    const moveit::core::RobotState* /*context_state*/) const
{
  // TODO
  using namespace drake::multibody;

  // TODO: This explicitly assumes that num_positions == num_velocities, which
  // would not be true in the case of e.g. quaternion axes. How does MoveIt
  // handle a reduced-DoF tangent space?
  const size_t N = joint_model_group_->getActiveVariableCount();

  Eigen::VectorXd q_current;
  current_state_->copyJointGroupPositions(joint_model_group_, q_current);

  Eigen::VectorXd v_current;
  current_state_->copyJointGroupVelocities(joint_model_group_, v_current);

  // TODO: Unclear the exact specifications of this Jacobian: w.r.t. what
  // reference frame? Analytical or geometric Jacobian? Seems to work OK but
  // should be clarified.
  const Eigen::MatrixXd& J = jacobian;

  // TODO: The DoDifferentialInverseKinematics function can handle many
  // addition costs and constraints, e.g.:
  // - joint position limits
  // - end-effector velocity/acceleration constraints
  // - nominal posture/centering cost
  DifferentialInverseKinematicsParameters params(N, N);

  // TODO: How do I get the real value from MoveIt?
  // params.set_time_step(0.01);

  // TODO: Hardcoded velocity and acceleration limits for demonstration
  // purposes. These should be drawn from the MoveIt/Servo configuration
  // instead.
  const Eigen::VectorXd v_max = Eigen::VectorXd::Constant(N, 1.0);
  const Eigen::VectorXd vdot_max = Eigen::VectorXd::Constant(N, 1.0);
  params.set_joint_velocity_limits({ -v_max, v_max });
  params.set_joint_acceleration_limits({ -vdot_max, vdot_max });

  const auto result = DoDifferentialInverseKinematics(q_current, v_current, delta_x, J, params);
  if (result.status == DifferentialInverseKinematicsStatus::kSolutionFound)
  {
    delta_theta_ = *result.joint_velocities;
  }
  else if (result.status == DifferentialInverseKinematicsStatus::kStuck)
  {
    // TODO: In this result joint velocities are still generated, they are
    // just not guaranteed to follow the Cartesian target velocity. Is it
    // acceptable to still use them? It may be the only way to get unstuck.
    RCLCPP_WARN(LOGGER, "Differential inverse kinematics is stuck!");
    return false;
  }
  else if (result.status == DifferentialInverseKinematicsStatus::kNoSolutionFound)
  {
    RCLCPP_WARN(LOGGER, "Differential inverse kinematics failed to converge to a solution!");
    return false;
  }
  RCLCPP_INFO(getLogger(), "IK Solver Succeeded!");
  return true;
}

bool DiffIKPlugin::getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                                 std::vector<geometry_msgs::msg::Pose>& poses) const
{
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    RCLCPP_ERROR(getLogger(), "Joint angles vector must have size: %ld", dimension_);
    return false;
  }

  // TODO
  RCLCPP_ERROR(getLogger(), "Forward kinematics not implemented");

  return false;
}
}  // namespace diff_ik_plugin
