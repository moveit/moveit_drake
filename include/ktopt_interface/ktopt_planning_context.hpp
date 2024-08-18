#pragma once

#include <moveit/planning_interface/planning_interface.h>
#include <ktopt_moveit_parameters.hpp>

// relevant drake includes
#include "drake/common/eigen_types.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h"
#include "drake/solvers/solve.h"
#include "drake/geometry/meshcat.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/geometry/meshcat_params.h"
#include "drake/geometry/geometry_frame.h"
#include "drake/geometry/geometry_instance.h"
#include "drake/geometry/geometry_roles.h"
#include "drake/geometry/proximity_properties.h"
#include "drake/visualization/visualization_config.h"
#include "drake/visualization/visualization_config_functions.h"
#include <drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h>

namespace ktopt_interface
{
// declare all namespaces to be used
using drake::geometry::AddRigidHydroelasticProperties;
using drake::geometry::Box;
using drake::geometry::Sphere;
using drake::geometry::Cylinder;
using drake::geometry::FrameId;
using drake::geometry::GeometryFrame;
using drake::geometry::GeometryId;
using drake::geometry::GeometryInstance;
using drake::geometry::IllustrationProperties;
using drake::geometry::Meshcat;
using drake::geometry::MeshcatParams;
using drake::geometry::MeshcatVisualizer;
using drake::geometry::MeshcatVisualizerParams;
using drake::geometry::PerceptionProperties;
using drake::geometry::ProximityProperties;
using drake::geometry::Role;
using drake::geometry::SceneGraph;
using drake::geometry::SourceId;
using drake::math::RigidTransformd;
using drake::multibody::AddMultibodyPlantSceneGraph;
using drake::multibody::MinimumDistanceLowerBoundConstraint;
using drake::multibody::MultibodyPlant;
using drake::multibody::PackageMap;
using drake::multibody::Parser;
using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;
using drake::solvers::Solve;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using drake::visualization::ApplyVisualizationConfig;
using drake::visualization::VisualizationConfig;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Joints = std::vector<const moveit::core::JointModel*>;

class KTOptPlanningContext : public planning_interface::PlanningContext
{
public:
  KTOptPlanningContext(const std::string& name, const std::string& group_name, const ktopt_interface::Params& params);

  void solve(planning_interface::MotionPlanResponse& res) override;
  void solve(planning_interface::MotionPlanDetailedResponse& res) override;

  bool terminate() override;
  void clear() override;

  void setRobotDescription(std::string robot_description);
  VectorXd toDrakePositions(const moveit::core::RobotState& state, const Joints& joints);
  void setJointPositions(const VectorXd& values, const Joints& joints, moveit::core::RobotState& state);
  void setJointVelocities(const VectorXd& values, const Joints& joints, moveit::core::RobotState& state);
  void transcribePlanningScene(const planning_scene::PlanningScene& planning_scene);
  RigidTransformd convertToDrakeFrame(const Eigen::Affine3d& ros_pose);

private:
  const ktopt_interface::Params params_;
  std::string robot_description_;

  // drake related variables
  std::unique_ptr<Diagram<double>> diagram_;
  std::unique_ptr<DiagramBuilder<double>> builder;
  std::unique_ptr<Context<double>> diagram_context_;
  VectorXd nominal_q_;
  const std::string OCTOMAP_NS = "<octomap>";

  // visualization
  std::shared_ptr<Meshcat> meshcat_;
  MeshcatVisualizer<double>* visualizer_;
};
}  // namespace ktopt_interface
