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

#include <moveit/planning_interface/planning_response_adapter.h>
#include <class_loader/class_loader.hpp>
#include <moveit/utils/logger.hpp>
//#include <moveit/drake/utils.hpp>

#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
//#include <default_response_adapter_parameters.hpp>

namespace drake_planning_response_adapters
{
/**
 * @brief Adapter to check the request path validity (collision avoidance, feasibility and constraint satisfaction)
 *
 */
class AddToppraParameterization : public planning_interface::PlanningResponseAdapter
{
public:
  AddToppraParameterization() : logger_(moveit::getLogger("moveit.ros.validate_solution"))
  {   
  }

  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override
  {
    auto param_listener =
        std::make_unique<toppra_parameters::ParamListener>(node, parameter_namespace);
    const auto params = param_listener->get_params();

  }

  [[nodiscard]] std::string getDescription() const override
  {
    return std::string("AddToppraParameterization");
  }

  void adapt(const planning_scene::PlanningSceneConstPtr& planning_scene,
             const planning_interface::MotionPlanRequest& req,
             planning_interface::MotionPlanResponse& res) const override
  {
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
    toppra_times = toppra.SolvePathParameterization()
  }

private:
  rclcpp::Logger logger_;
  drake::multibody::MultibodyPlant plant_;
  drake::SceneGraph<double>* scene_graph_;
};
}  // namespace drake_planning_response_adapters

CLASS_LOADER_REGISTER_CLASS(drake_planning_response_adapters::AddToppraParameterization,
                            planning_interface::PlanningResponseAdapter)
