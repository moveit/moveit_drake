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

#include <moveit/planning_interface/planning_response_adapter.h>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>

#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"

// #include <toppra_parameters.hpp>

namespace moveit::drake
{

/**
 * @brief TODO
 *
 */
class AddToppraTimeParameterization : public planning_interface::PlanningResponseAdapter
{
public:
  AddToppraTimeParameterization() : logger_(moveit::getLogger("moveit.drake.add_toppra_time_parameterization"))
  {
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    // TODO
    // param_listener_ = std::make_unique<toppra_parameters::ParamListener>(node, parameter_namespace);
  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("AddToppraTimeParameterization");
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& /*planning_scene*/,
             const planning_interface::MotionPlanRequest& req,
             planning_interface::MotionPlanResponse& res) const override
  {
    // Check if res contains a path
    if (!res.trajectory)
    {
      RCLCPP_ERROR(logger_, "Cannot apply response adapter '%s' because MotionPlanResponse does not contain a path.",
                   getDescription().c_str());
      res.error_code = moveit::core::MoveItErrorCode::INVALID_MOTION_PLAN;
      return;
    }

    /*
    // Add a multibody plant and a scene graph to hold the robots
    drake::multibody::MultibodyPlantConfig plant_config;
    plant_config.time_step = 0.001;
    plant_config.discrete_contact_solver = "sap";
    auto [plant_, scene_graph_] =
        drake::multibody::AddMultibodyPlant(plant_config, &builder);
    // Create plant
    auto parser = drake::multibody::Parser(&plant);
    auto model_instance = parser.AddModelsFromString(params.robot_description);

    // Weld the robot to the world so it doesn't fall through the floor
    auto& base_frame = plant.GetFrameByName("base", model_instance);
    auto X_WB = drake::math::RigidTransform(drake::Vector3<double>{
        static_cast<double>(xx), static_cast<double>(yy), 0.});
    plant.WeldFrames(plant.world_frame(), base_frame, X_WB);

    // TODO Add collision information

    plant.Finalize();


    // Run toppra (TODO)
    grid_points = Toppra.CalcGridPoints(gcs_traj, CalcGridPointsOptions())
    toppra = Toppra(gcs_traj, plant, grid_points)
    toppra.AddJointVelocityLimit(velocity_lb, velocity_ub)
    toppra.AddJointAccelerationLimit(accel_lb, accel_ub)
    toppra_times = toppra.SolvePathParameterization()*/

    if (false /* TODO*/)
    {
      res.error_code = moveit::core::MoveItErrorCode::SUCCESS;
    }
    else
    {
      RCLCPP_ERROR(logger_, "Response adapter '%s' failed to generate a trajectory.", getDescription().c_str());
      res.error_code = moveit::core::MoveItErrorCode::FAILURE;
    }
  }

protected:
  //std::unique_ptr<default_response_adapter_parameters::ParamListener> param_listener_;
  rclcpp::Logger logger_;
};

}  // namespace moveit::drake

CLASS_LOADER_REGISTER_CLASS(moveit::drake::AddToppraTimeParameterization,
                            planning_interface::PlanningResponseAdapter)
