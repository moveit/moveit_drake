#pragma once

#include <ktopt_moveit_parameters.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <shape_msgs/msg/solid_primitive.h>

// relevant drake includes
#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/planning/trajectory_optimization/kinematic_trajectory_optimization.h>
#include <drake/systems/framework/diagram.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h>
#include <drake/multibody/inverse_kinematics/position_constraint.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace ktopt_interface
{
// declare all namespaces to be used
using drake::multibody::MinimumDistanceLowerBoundConstraint;
using drake::multibody::MultibodyPlant;
using drake::multibody::PositionConstraint;
using drake::planning::trajectory_optimization::KinematicTrajectoryOptimization;
using drake::systems::Context;
using drake::systems::Diagram;
using drake::systems::DiagramBuilder;
using Joints = std::vector<const moveit::core::JointModel*>;

/**
 * @brief Helper class that defines a planning context for Drake Kinematic Trajectory Optimization (KTOpt).
 * @details For more information, refer to the Drake documentation:
 * https://drake.mit.edu/doxygen_cxx/classdrake_1_1planning_1_1trajectory__optimization_1_1_kinematic_trajectory_optimization.html
 */
class KTOptPlanningContext : public planning_interface::PlanningContext
{
public:
  /**
   * @brief Constructs an instance of a KTOpt plannign context.
   * @param name The name of the planning context.
   * @param group_name The name of the joint group used for motion planning.
   * @param params The ROS parameters for this planner.
   */
  KTOptPlanningContext(const std::string& name, const std::string& group_name, const ktopt_interface::Params& params);

  /**
   * @brief Calculates a trajectory for the current request of this context.
   * @param res The result containing the respective trajectory, or error code on failure.
   */
  void solve(planning_interface::MotionPlanResponse& res) override;

  /**
   * @brief Calculates a trajectory for the current request of this context.
   * @details This function just delegates to the common response.
   * However, here the same trajectory is stored with the descriptions "plan","simplify", or "interpolate".
   * @param res The detailed result containing the respective trajectory, or error code on failure.
   */
  void solve(planning_interface::MotionPlanDetailedResponse& res) override;

  /**
   * @brief Terminates any running solutions.
   * @return True if successful, otherwise false.
   */
  bool terminate() override;

  /// @brief Clear the data structures used by the planner.
  void clear() override;

  /**
   * @brief Sets the current robot description for planning.
   * @param robot_description The URDF string containing the robot description.
   */
  void setRobotDescription(const std::string& robot_description);

  /**
   * @brief Transcribes a MoveIt planning scene to the Drake multibody plant used by this planner.
   * @param planning_scene The MoveIt planning scene to transcribe.
   */
  void transcribePlanningScene(const planning_scene::PlanningScene& planning_scene);

  /**
   * @brief Adds path position constraints, if any, to the planning problem.
   * @param trajopt The Drake mathematical program containing the trajectory optimization problem.
   * @param plant The Drake multibody plant to use for planning.
   * @param plant_context The context associated with the multibody plant.
   * @param padding Additional position padding on the MoveIt constraint, in meters.
   * This ensures that constraints are more likely to hold for the entire trajectory, since the
   * Drake mathematical program only optimizes constraints at discrete points along the path.
   */
  void addPathPositionConstraints(KinematicTrajectoryOptimization& trajopt, const MultibodyPlant<double>& plant,
                                  Context<double>& plant_context, const double padding);

private:
  /// @brief The ROS parameters associated with this motion planner.
  const ktopt_interface::Params params_;

  /// @brief The URDF robot description.
  std::string robot_description_;

  /// @brief The Drake diagram describing the entire system.
  std::unique_ptr<Diagram<double>> diagram_;

  /// @brief The builder for the Drake system diagram.
  std::unique_ptr<DiagramBuilder<double>> builder;

  /// @brief The context that contains all the data necessary to perform computations on the diagram.
  std::unique_ptr<Context<double>> diagram_context_;

  /// @brief The nominal joint configuration of the robot, used for joint centering objectives.
  Eigen::VectorXd nominal_q_;

  /// @brief Pointer to the Meshcat instance associated with this planner.
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;

  /// @brief The Drake MeshCat visualizer associated with this planner.
  drake::geometry::MeshcatVisualizer<double>* visualizer_;
};
}  // namespace ktopt_interface
