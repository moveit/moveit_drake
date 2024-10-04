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
   Description: Add time parameterization with Drake's TOPPRA implementation.
*/

#include <moveit/drake/conversions.hpp>
#include <moveit/planning_interface/planning_response_adapter.h>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>

#include <drake/multibody/parsing/parser.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/optimization/toppra.h>

/* Visualization */
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_params.h"
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

using ::drake::geometry::Meshcat;
using ::drake::geometry::MeshcatParams;
using ::drake::geometry::MeshcatVisualizer;
using ::drake::geometry::MeshcatVisualizerParams;

namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.drake.toppra");
}
}  // namespace
/**
 * @brief Post-processing adapter that timeparametermizes a trajectory based on reachability analysis. For details see
 * https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_toppra.html
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
                           "urdf/panda_arm_hand.urdf";
    const std::string urdf = PackageMap{}.ResolveUrl(ModelUrl);
    Parser(&plant, &scene_graph).AddModels(urdf);
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));

    // for now finalize plant here
    plant.Finalize();

    const auto meshcat_params = MeshcatParams();
    meshcat_ = std::make_shared<Meshcat>(meshcat_params);

    MeshcatVisualizerParams meshcat_viz_params;
    auto& visualizer =
        MeshcatVisualizer<double>::AddToBuilder(builder.get(), scene_graph, meshcat_, std::move(meshcat_viz_params));
    visualizer_ = &visualizer;

    diagram_ = builder->Build();
    diagram_context_ = diagram_->CreateDefaultContext();
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("AddToppraTimeParameterization");
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
             const planning_interface::MotionPlanRequest& /*req*/,
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
    auto& plant = diagram_->GetDowncastSubsystemByName<MultibodyPlant<double>>("plant");
    auto& plant_context = diagram_->GetMutableSubsystemContext(plant, diagram_context_.get());
    Eigen::VectorXd q = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
    Eigen::VectorXd joint_positions =
        moveit::drake::getJointPositionVector(res.trajectory->getFirstWayPoint(), joint_model_group->getName(), plant);
    Eigen::VectorXd joint_velocities =
        moveit::drake::getJointVelocityVector(res.trajectory->getFirstWayPoint(), joint_model_group->getName(), plant);
    q << joint_positions, joint_velocities;
    plant.SetPositionsAndVelocities(&plant_context, q);

    // Create drake::trajectories::Trajectory from moveit trajectory
    auto input_trajectory = getPiecewisePolynomial(*res.trajectory, joint_model_group, plant);

    // Run toppra
    const auto grid_points = Toppra::CalcGridPoints(input_trajectory, CalcGridPointsOptions());
    auto toppra = Toppra(input_trajectory, plant, grid_points);

    // Get velocity and acceleration bounds
    Eigen::VectorXd lower_velocity_limits;
    Eigen::VectorXd upper_velocity_limits;
    Eigen::VectorXd lower_acceleration_limits;
    Eigen::VectorXd upper_acceleration_limits;

    getVelocityBounds(joint_model_group, plant, lower_velocity_limits, upper_velocity_limits);
    getAccelerationBounds(joint_model_group, plant, lower_acceleration_limits, upper_acceleration_limits);

    toppra.AddJointVelocityLimit(lower_velocity_limits, upper_velocity_limits);
    toppra.AddJointAccelerationLimit(lower_acceleration_limits, upper_acceleration_limits);
    auto optimized_trajectory = toppra.SolvePathParameterization();

    if (!optimized_trajectory.has_value())
    {
      RCLCPP_ERROR_STREAM(getLogger(), "Failed to calculate a trajectory with toppra");
      res.error_code = moveit::core::MoveItErrorCode::FAILURE;
      return;
    }

    //getRobotTrajectory(optimized_trajectory.value(),
    //                   optimized_trajectory.value().end_time() /
    //                       res.trajectory->getWayPointCount() /* TODO enable down sampling */,
    //                   plant, res.trajectory /* override previous solution with optimal trajectory*/);

    // meshcat experiment
    auto& vis_context = visualizer_->GetMyContextFromRoot(*diagram_context_);
    visualizer_->ForcedPublish(vis_context);

    // Visualize the trajectory with Meshcat
    visualizer_->StartRecording();
    const auto num_pts = res.trajectory->getWayPointCount();
    RCLCPP_INFO_STREAM(getLogger(), "print trajectory");
    for (unsigned int i = 0; i < num_pts; ++i)
    {
      const auto t_scale = static_cast<double>(i) / static_cast<double>(num_pts - 1);
      const auto t = std::min(t_scale, 1.0) * optimized_trajectory.value().end_time();
      RCLCPP_INFO_STREAM(getLogger(), "Positions size: " << optimized_trajectory.value().value(t).size());
      plant.SetPositions(&plant_context, optimized_trajectory.value().value(t));
      auto& vis_context = visualizer_->GetMyContextFromRoot(*diagram_context_);
      visualizer_->ForcedPublish(vis_context);
      // Without these sleeps, the visualizer won't give you time to load your browser
      std::this_thread::sleep_for(std::chrono::milliseconds(10000));
    }
    visualizer_->StopRecording();
    visualizer_->PublishRecording();

    res.error_code = moveit::core::MoveItErrorCode::SUCCESS;
  }

protected:
  // std::unique_ptr<default_response_adapter_parameters::ParamListener> param_listener_;
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<Context<double>> diagram_context_;

  // Temporary visualization
  std::shared_ptr<Meshcat> meshcat_;
  MeshcatVisualizer<double>* visualizer_;
};

}  // namespace moveit::drake

CLASS_LOADER_REGISTER_CLASS(moveit::drake::AddToppraTimeParameterization, planning_interface::PlanningResponseAdapter)
