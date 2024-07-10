#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <ktopt_moveit_parameters.hpp>

// relevant drake includes
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"
#include "drake/solvers/solve.h"

namespace ktopt_interface
{
// declare all namespaces to be used
using drake::multibody::MultibodyPlant;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::geometry::SceneGraph;
using drake::systems::DiagramBuilder;
using drake::multibody::Parser;
using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;
using drake::solvers::Solve;
using drake::systems::Diagram;
using drake::systems::Context;

class KTOptPlanningContext : public planning_interface::PlanningContext
{
public:
    KTOptPlanningContext(
        const std::string& name,
        const std::string& group_name,
        const ktopt_interface::Params& params);

    void solve(
        planning_interface::MotionPlanResponse& res) override;
    void solve(
        planning_interface::MotionPlanDetailedResponse& res) override;

    bool terminate() override;
    void clear() override;

    void setRobotDescription(std::string robot_description);

private:
    const ktopt_interface::Params params_;
    std::string robot_description_;

    // drake related variables
    SceneGraph<double>* scene_graph_{};
    MultibodyPlant<double>* plant_{};
    std::unique_ptr<Context<double>> diagram_context_;


};
} // namespace ktopt_interface
