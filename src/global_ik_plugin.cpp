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

// register SRVKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(diff_ik_plugin::DifferentialIKPlugin, kinematics::KinematicsBase)

namespace diff_ik_plugin
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.kinematics.diff_ik_plugin");
}
}  // namespace

DifferentialIKPlugin::DifferentialIKPlugin() : active_(false)
{
}

bool DifferentialIKPlugin::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModel& robot_model,
                                     const std::string& group_name, const std::string& base_frame,
                                     const std::vector<std::string>& tip_frames, double search_discretization)
{
  // TODO
  return true;
}

bool DifferentialIKPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  // TODO
  return true;
}

bool DifferentialIKPlugin::timedOut(const rclcpp::Time& start_time, double duration) const
{
  return ((node_->now() - start_time).seconds() >= duration);
}

bool DifferentialIKPlugin::getPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                        const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                        moveit_msgs::msg::MoveItErrorCodes& error_code,
                                        const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, consistency_limits, solution, IKCallbackFn(),
                          error_code, options);
}

bool DifferentialIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool DifferentialIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, IKCallbackFn(), error_code,
                          options);
}

bool DifferentialIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool DifferentialIKPlugin::searchPositionIK(const geometry_msgs::msg::Pose& ik_pose,
                                           const std::vector<double>& ik_seed_state, double timeout,
                                           const std::vector<double>& consistency_limits, std::vector<double>& solution,
                                           const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::msg::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool DifferentialIKPlugin::searchPositionIK(const std::vector<geometry_msgs::msg::Pose>& ik_poses,
                                           const std::vector<double>& ik_seed_state, double /*timeout*/,
                                           const std::vector<double>& /*consistency_limits*/,
                                           std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                           moveit_msgs::msg::MoveItErrorCodes& error_code,
                                           const kinematics::KinematicsQueryOptions& /*options*/,
                                           const moveit::core::RobotState* /*context_state*/) const
{
  // Check if active
  if (!active_)
  {
    RCLCPP_ERROR(getLogger(), "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check if seed state correct
  if (ik_seed_state.size() != dimension_)
  {
    RCLCPP_ERROR_STREAM(getLogger(),
                        "Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    RCLCPP_ERROR_STREAM(getLogger(), "Mismatched number of pose requests (" << ik_poses.size() << ") to tip frames ("
                                                                            << tip_frames_.size()
                                                                            << ") in searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // TODO
  RCLCPP_INFO(getLogger(), "IK Solver Succeeded!");
  return true;
}

bool DifferentialIKPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                        const std::vector<double>& joint_angles,
                                        std::vector<geometry_msgs::msg::Pose>& poses) const
{
  if (!active_)
  {
    RCLCPP_ERROR(getLogger(), "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    RCLCPP_ERROR(getLogger(), "Joint angles vector must have size: %d", dimension_);
    return false;
  }

  // TODO
  RCLCPP_ERROR(getLogger(), "Forward kinematics not implemented");

  return false;
}
}  // namespace diff_ik_plugin
