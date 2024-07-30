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

#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include <drake/common/trajectories/trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
namespace moveit::drake
{

using ::drake::geometry::SceneGraph;
using ::drake::multibody::AddMultibodyPlantSceneGraph;
using ::drake::multibody::MultibodyPlant;
using ::drake::multibody::Parser;
using ::drake::systems::DiagramBuilder;

/**
 * @brief 
 * 
 * @param robot_trajectory 
 * @return drake::trajectories::Trajectory<double> 
 */
[[nodiscard]] ::drake::trajectories::PiecewisePolynomial<double> getPiecewisePolynomial(const ::robot_trajectory::RobotTrajectory& robot_trajectory) {

    std::vector<double> breaks;
    std::vector<Eigen::MatrixXd> samples;

    const auto& waypoints = robot_trajectory.getWayPointDurations();
    double time = 0.0;
    for (std::size_t i = 0; i < robot_trajectory.getWayPointCount(); ++i) {
        time += waypoints[i];
        breaks.push_back(time);
        
        const auto& state = robot_trajectory.getWayPoint(i);
        Eigen::VectorXd position(state.getVariableCount());
        for (std::size_t j = 0; j < state.getVariableCount(); ++j) {
            position[j] = state.getVariablePosition(j);
        }
        samples.emplace_back(position);
    }

    // Create a piecewise polynomial trajectory
    return ::drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(breaks, samples);
}

void getRobotTrajectory(const ::drake::trajectories::PiecewisePolynomial<double>& piecewise_polynomial, const int samples, std::shared_ptr<::robot_trajectory::RobotTrajectory>& output_trajectory) {

  // Get the start and end times of the piecewise polynomial
  const auto time_step = piecewise_polynomial.end_time() / static_cast<double>(samples - 1);

  for (double t = 0.0; t <= piecewise_polynomial.end_time(); t += time_step)
  {
    const auto pos_val = piecewise_polynomial.value(t);
    const auto vel_val = piecewise_polynomial.EvalDerivative(t);
    const auto waypoint = std::make_shared<moveit::core::RobotState>(output_trajectory->getRobotModel());
    const auto active_joints = output_trajectory->getRobotModel()->getActiveJointModels();
    for (size_t joint_index = 0; joint_index < active_joints.size(); ++joint_index)
    {
      waypoint->setJointPositions(active_joints[joint_index], &pos_val(joint_index));
      waypoint->setJointVelocities(active_joints[joint_index], &vel_val(joint_index));
    }

    output_trajectory->addSuffixWayPoint(waypoint, time_step);
  }
}

// TODO consider creating a struct
//::drake::multibody::MultibodyPlant<double> getMultiBodyPlant(
//    const ::planning_scene::PlanningScene& planning_scene, const std::string& urdf) {
//
//    SceneGraph<double>* scene_graph{};
//    MultibodyPlant<double>* plant{};
//    DiagramBuilder<double> builder;
//    std::tie(plant, scene_graph) = AddMultibodyPlantSceneGraph(&builder, 0.0);
//
//    auto robot_instance = Parser(plant, scene_graph).AddModels(urdf);
//    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("panda_link0"));
//
//
//    plant->Finalize();
//
//    return *plant;
//}
}  // namespace moveit::drake
