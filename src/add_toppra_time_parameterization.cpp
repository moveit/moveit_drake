/*********************************************************************
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2024, PickNik Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
   Description: TODO
*/

#include <moveit/drake/conversions.hpp>
#include <moveit/planning_interface/planning_response_adapter.h>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>

#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include <drake/multibody/optimization/toppra.h>
// #include <toppra_parameters.hpp>

namespace moveit::drake
{
using ::drake::geometry::SceneGraph;
using ::drake::multibody::AddMultibodyPlantSceneGraph;
using ::drake::multibody::CalcGridPointsOptions;
using ::drake::multibody::MultibodyPlant;
using ::drake::multibody::PackageMap;
using ::drake::multibody::Parser;
using ::drake::multibody::Toppra;
using ::drake::systems::Context;
using ::drake::systems::Diagram;
using ::drake::systems::DiagramBuilder;

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.drake.toppra");
}
}  // namespace
/**
 * @brief TODO
 *
 */
class AddToppraTimeParameterization : public planning_interface::PlanningResponseAdapter
{
public:
  AddToppraTimeParameterization() = default;

  void initialize(const rclcpp::Node::SharedPtr& /*node*/, const std::string& /*parameter_namespace*/) override
  {
    // TODO
    // param_listener_ = std::make_unique<toppra_parameters::ParamListener>(node, parameter_namespace);

    // Construct diagram
    auto builder = std::make_unique<DiagramBuilder<double>>();

    auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(builder.get(), 0.0 /* model as continuous system */);

    // TODO(sjahr) Replace with subscribed robot description
    const char* ModelUrl = "package://drake_models/franka_description/"
                           "urdf/panda_arm.urdf";
    const std::string urdf = PackageMap{}.ResolveUrl(ModelUrl);
    Parser(&plant, &scene_graph).AddModels(urdf);
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));

    // for now finalize plant here
    plant.Finalize();

    diagram_ = builder->Build();
    diagram_context_ = diagram_->CreateDefaultContext();
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("AddToppraTimeParameterization");
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& /* req */,
             planning_interface::MotionPlanResponse& res) const override
  {
    // Check if res contains a path
    if (!res.trajectory)
    {
      RCLCPP_ERROR(getLogger(),
                   "Cannot apply response adapter '%s' because MotionPlanResponse does not contain a path.",
                   getDescription().c_str());
      res.error_code = moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN;
      return;
    }

    // Get joint model group
    const moveit::core::JointModelGroup* joint_model_group = res.trajectory->getGroup();
    if (!joint_model_group)
    {
      RCLCPP_ERROR(getLogger(), "It looks like the pipeline did not set the group the plan was computed for");
      res.error_code = moveit::core::MoveItErrorCode::FAILURE;
      return;
    }

    /////////////////////////////////////
    // Update plan from planning scene //
    /////////////////////////////////////
    auto& plant = dynamic_cast<const MultibodyPlant<double>&>(diagram_->GetSubsystemByName("plant"));
    auto& plant_context = diagram_->GetMutableSubsystemContext(plant, diagram_context_.get());
    const auto& current_state = planning_scene->getCurrentState();
    Eigen::VectorXd q_pos = Eigen::VectorXd::Zero(joint_model_group->getActiveVariableCount());
    Eigen::VectorXd q_vel = Eigen::VectorXd::Zero(joint_model_group->getActiveVariableCount());
    Eigen::VectorXd q = Eigen::VectorXd::Zero(2 * joint_model_group->getActiveVariableCount());

    current_state.copyJointGroupPositions(joint_model_group, q_pos);
    current_state.copyJointGroupVelocities(joint_model_group, q_vel);
    q << q_pos;
    q << q_vel;
    plant.SetPositionsAndVelocities(&plant_context, q);

    // Create drake::trajectories::Trajectory from moveit trajectory
    auto input_trajectory = getPiecewisePolynomial(*res.trajectory);
    // Run toppra (TODO)
    const auto grid_points = Toppra::CalcGridPoints(input_trajectory, CalcGridPointsOptions());
    auto toppra = Toppra(input_trajectory, plant, grid_points);

    /////////////////////////////////////////////////////////////////////////
    // Read joint bounds from robot model (TODO(sjahr): Expose in MoveIt2) //
    /////////////////////////////////////////////////////////////////////////
    // Get the velocity and acceleration limits for all active joints
    const moveit::core::RobotModel& robot_model = joint_model_group->getParentModel();
    const std::vector<std::string>& joint_variables = joint_model_group->getVariableNames();
    std::vector<size_t> active_joint_indices;
    if (!joint_model_group->computeJointVariableIndices(joint_model_group->getActiveJointModelNames(),
                                                        active_joint_indices))
    {
      RCLCPP_ERROR(getLogger(), "Failed to get active variable indices.");
    }

    const size_t num_active_joints = active_joint_indices.size();
    Eigen::VectorXd min_velocity(num_active_joints);
    Eigen::VectorXd max_velocity(num_active_joints);
    Eigen::VectorXd min_acceleration(num_active_joints);
    Eigen::VectorXd max_acceleration(num_active_joints);
    for (size_t idx = 0; idx < num_active_joints; ++idx)
    {
      // For active joints only (skip mimic joints and other types)
      const moveit::core::VariableBounds& bounds =
          robot_model.getVariableBounds(joint_variables[active_joint_indices[idx]]);

      // Limits need to be non-zero, otherwise we never exit
      if (bounds.velocity_bounded_)
      {
        max_velocity[idx] = bounds.max_velocity_;  // TODO(sjahr) consider scaling factor
        min_velocity[idx] = bounds.min_velocity_;  // TODO(sjahr) consider scaling factor
      }
      else
      {
        RCLCPP_ERROR_STREAM(getLogger(), "No velocity limit was defined for joint " << joint_variables.at(idx).c_str()
                                                                                    << "! You have to define velocity "
                                                                                       "limits "
                                                                                       "in the URDF or "
                                                                                       "joint_limits.yaml");
        res.error_code = moveit::core::MoveItErrorCode::FAILURE;
        return;
      }

      if (bounds.acceleration_bounded_)
      {
        min_acceleration[idx] = bounds.min_acceleration_;  // TODO(sjahr) consider scaling factor
        max_acceleration[idx] = bounds.max_acceleration_;  // TODO(sjahr) consider scaling factor
      }
      else
      {
        RCLCPP_ERROR_STREAM(getLogger(), "No acceleration limit was defined for joint "
                                             << joint_variables.at(idx).c_str()
                                             << "! You have to define "
                                                "acceleration "
                                                "limits in the URDF or "
                                                "joint_limits.yaml");
        res.error_code = moveit::core::MoveItErrorCode::FAILURE;
        return;
      }
    }

    //
    toppra.AddJointVelocityLimit(max_velocity, min_velocity);
    toppra.AddJointAccelerationLimit(min_acceleration, max_acceleration);
    auto optimized_trajectory = toppra.SolvePathParameterization();

    if (!optimized_trajectory.has_value())
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Failed to calculate a trajectory with toppra");
      res.error_code = moveit::core::MoveItErrorCode::FAILURE;
      return;
    }

    getRobotTrajectory(optimized_trajectory.value(), res.trajectory->getWayPointCount() /* TODO enable down sampling */,
                       res.trajectory /* override previous solution with optimal trajectory*/);
    res.error_code = moveit::core::MoveItErrorCode::SUCCESS;
  }

protected:
  // std::unique_ptr<default_response_adapter_parameters::ParamListener> param_listener_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> diagram_context_;
};

}  // namespace moveit::drake

CLASS_LOADER_REGISTER_CLASS(moveit::drake::AddToppraTimeParameterization, planning_interface::PlanningResponseAdapter)
