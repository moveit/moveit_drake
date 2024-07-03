#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/planning_scene/planning_scene.h>

#include "ktopt_interface/ktopt_planning_context.hpp"

namespace ktopt_interface
{
namespace
{
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.ktopt_drake.planning_context");
}
}  // namespace

KTOptPlanningContext::KTOptPlanningContext(const std::string& name, const std::string& group_name,
                                           const ktopt_interface::Params& params)
  : planning_interface::PlanningContext(name, group_name), params_(params)
{
  // Do some drake initialization may be
}

void KTOptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& res)
{
  RCLCPP_ERROR(getLogger(),
               "KTOptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse&) is not implemented!");
  return;
}

void KTOptPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  // preliminary house keeping
  auto time_start = std::chrono::steady_clock::now();
  res.planner_id = std::string("ktopt");
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  // Retrieve motion plan request
  const auto& req = getMotionPlanRequest();
  const moveit::core::RobotState start_state(*getPlanningScene()->getCurrentStateUpdated(req.start_state));
  // TODO: update plant_ state here as well

  // retrieve goal state
  moveit::core::RobotState goal_state(start_state);
  constraint_samplers::ConstraintSamplerManager sampler_manager;
  auto goal_sampler = sampler_manager.selectSampler(getPlanningScene(), getGroupName(), req.goal_constraints[0]);
  if (!goal_sampler || !goal_sampler->sample(goal_state))
  {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS;
    return;
  }

  // compile into a Kinematic Trajectory Optimization problem
  auto trajopt = KinematicTrajectoryOptimization(plant_->num_positions(), 10);

  // bare minimum specification
  // TODO: Add constraints on start joint configuration
  // TODO: Add constraint on end joint configuration or pose
  // TODO: Add constraints on joint position/velocity/acceleration
  // TODO: Add collision checking distance constraints
  // TODO: Add position/orientation constraints, if any specified in the motion planning request

  // solve the program
  auto& prog = trajopt.get_mutable_prog();
  auto result = Solve(prog);

  if (!result.is_success())
  {
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  // package up the resulting trajectory
  auto traj = trajopt.ReconstructTrajectory(result);
  const size_t num_pts = 101;  // TODO: should be sample time based instead
  const auto time_step = traj.end_time() / static_cast<double>(num_pts - 1);
  for (double t = 0.0; t <= traj.end_time(); t += time_step)
  {
    const auto val = traj.value(t);
    // TODO: Put into the robot trajectory in the response.
  }
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return;
}

bool KTOptPlanningContext::terminate()
{
  return true;
}

void KTOptPlanningContext::setRobotDescription(std::string robot_description)
{
  robot_description_ = robot_description;

  // also perform some drake related initialisations here
  DiagramBuilder<double> builder;

  std::tie(plant_, scene_graph_) = AddMultibodyPlantSceneGraph(&builder, 0.0);

  auto robot_instance = Parser(plant_, scene_graph_).AddModels(robot_description_);
  plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("panda_link0"));

  // for now finalize plant here
  plant_->Finalize();

  // in the future you can add other LeafSystems here. For now building the
  // diagram
  auto diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  Context<double>& plant_context_ = diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
}

void KTOptPlanningContext::clear()
{
  // do something
}
}  // namespace ktopt_interface
