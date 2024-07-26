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
  return moveit::getLogger("moveit.planners.ktopt_interface.planning_context");
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
  RCLCPP_INFO(getLogger(), "Setting up and solving optimization problem ...");
  // preliminary house keeping
  const auto time_start = std::chrono::steady_clock::now();
  res.planner_id = std::string("ktopt");
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  // Retrieve motion plan request
  const auto& req = getMotionPlanRequest();
  const moveit::core::RobotState start_state(*getPlanningScene()->getCurrentStateUpdated(req.start_state));
  const auto group = getPlanningScene()->getRobotModel()->getJointModelGroup(getGroupName());
  RCLCPP_INFO_STREAM(getLogger(), "Planning for group: " << getGroupName());
  const auto& joints = group->getActiveJointModels();

  // q represents the complete state (joint positions and velocities)
  const auto q_p = toDrakePositions(start_state, joints);
  VectorXd q_v = VectorXd::Zero(joints.size());
  VectorXd q = VectorXd::Zero(2 * joints.size());
  q << q_p;
  q << q_v;

  // drake accepts a VectorX<T>
  plant_->SetPositionsAndVelocities(plant_context_, q);

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
  auto trajopt = KinematicTrajectoryOptimization(plant_->num_positions(), params_.control_points);
  auto& prog = trajopt.get_mutable_prog();

  // Costs
  trajopt.AddDurationCost(1.0);
  trajopt.AddPathLengthCost(1.0);
  // TODO: Adds quadratic cost
  // This acts as a secondary cost to push the solution towards joint centers
  // prog.AddQuadraticErrorCost(
  //   MatrixXd::Identity(joints.size(), joints.size()),
  //   nominal_q_;
  // );
  // prog.AddQuadraticErrorCost();

  // Constraints
  // Add constraints on start joint configuration and velocity
  trajopt.AddPathPositionConstraint(toDrakePositions(start_state, joints), toDrakePositions(start_state, joints), 0.0);
  trajopt.AddPathVelocityConstraint(VectorXd::Zero(joints.size()), VectorXd::Zero(joints.size()), 0.0);
  // Add constraint on end joint configuration and velocity
  trajopt.AddPathPositionConstraint(toDrakePositions(goal_state, joints), toDrakePositions(goal_state, joints), 1.0);
  trajopt.AddPathVelocityConstraint(VectorXd::Zero(joints.size()), VectorXd::Zero(joints.size()), 1.0);
  // TODO Add constraints on joint position/velocity/acceleration
  // trajopt.AddPositionBounds(
  //     plant_->GetPositionLowerLimits(),
  //     plant_->GetPositionUpperLimits());
  // trajopt.AddVelocityBounds(
  //     plant_->GetVelocityLowerLimits(),
  //     plant_->GetVelocityUpperLimits());

  // Add constraints on duration
  trajopt.AddDurationConstraint(0.5, 5);

  // TODO: Add collision checking distance constraints
  // TODO: Add position/orientation constraints, if any specified in the motion planning request

  // solve the program
  auto result = Solve(prog);

  if (!result.is_success())
  {
    RCLCPP_ERROR(getLogger(), "Trajectory optimization failed");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  // package up the resulting trajectory
  auto traj = trajopt.ReconstructTrajectory(result);
  const size_t num_pts = params_.trajectory_res;  // TODO: should be sample time based instead
  const auto time_step = traj.end_time() / static_cast<double>(num_pts - 1);
  res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(start_state.getRobotModel(), group);
  res.trajectory->clear();
  const auto& active_joints = res.trajectory->getGroup() ? res.trajectory->getGroup()->getActiveJointModels() :
                                                           res.trajectory->getRobotModel()->getActiveJointModels();

  // sanity check
  assert(traj.rows() == active_joints.size());

  visualizer_->StartRecording();
  for (double t = 0.0; t <= traj.end_time(); t += time_step)
  {
    const auto pos_val = traj.value(t);
    const auto vel_val = traj.EvalDerivative(t);
    const auto waypoint = std::make_shared<moveit::core::RobotState>(start_state);
    setJointPositions(pos_val, active_joints, *waypoint);
    setJointVelocities(vel_val, active_joints, *waypoint);
    res.trajectory->addSuffixWayPoint(waypoint, time_step);

    plant_->SetPositions(plant_context_, pos_val);
    auto& vis_context = visualizer_->GetMyContextFromRoot(*diagram_context_);
    visualizer_->ForcedPublish(vis_context);

    // Without these sleeps, the visualizer won't give you time to load your browser
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  visualizer_->StopRecording();
  visualizer_->PublishRecording();
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return;
}

bool KTOptPlanningContext::terminate()
{
  RCLCPP_ERROR(getLogger(), "KTOptPlanningContext::terminate() is not implemented!");
  return true;
}

void KTOptPlanningContext::setRobotDescription(std::string robot_description)
{
  robot_description_ = robot_description;

  // also perform some drake related initialisations here
  DiagramBuilder<double> builder;

  // meshcat experiment
  const auto meshcat_params = MeshcatParams();
  meshcat_ = std::make_shared<Meshcat>(meshcat_params);

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(&builder, 0.0);
  plant_ = &plant;
  scene_graph_ = &scene_graph;

  // TODO:(kamiradi) Figure out object parsing
  // auto robot_instance = Parser(plant_, scene_graph_).AddModelsFromString(robot_description_, ".urdf");

  // HACK: For now loading directly from drake's package map
  const char* ModelUrl = "package://drake_models/franka_description/"
                         "urdf/panda_arm.urdf";
  const std::string urdf = PackageMap{}.ResolveUrl(ModelUrl);
  auto robot_instance = Parser(plant_, scene_graph_).AddModels(urdf);
  plant_->WeldFrames(plant_->world_frame(), plant_->GetFrameByName("panda_link0"));

  // for now finalize plant here
  plant_->Finalize();

  // Apply MeshCat visualization
  VisualizationConfig config;
  ApplyVisualizationConfig(config, &builder, /*lcm_buses*/ nullptr, plant_, scene_graph_, meshcat_);

  MeshcatVisualizerParams meshcat_viz_params;
  auto& visualizer =
      MeshcatVisualizer<double>::AddToBuilder(&builder, *scene_graph_, meshcat_, std::move(meshcat_viz_params));
  visualizer_ = &visualizer;

  // in the future you can add other LeafSystems here. For now building the
  // diagram
  diagram_ = builder.Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  auto& plant_context = diagram_->GetMutableSubsystemContext(*plant_, diagram_context_.get());
  plant_context_ = &plant_context;

  nominal_q_ = plant_->GetPositions(*plant_context_);

  auto& vis_context = visualizer_->GetMyContextFromRoot(*diagram_context_);
  visualizer_->ForcedPublish(vis_context);
}

VectorXd KTOptPlanningContext::toDrakePositions(const moveit::core::RobotState& state, const Joints& joints)
{
  VectorXd q = VectorXd::Zero(joints.size());
  for (size_t joint_index = 0; joint_index < joints.size(); ++joint_index)
  {
    q[joint_index] = *state.getJointPositions(joints[joint_index]);
  }
  return q;
}

void KTOptPlanningContext::setJointPositions(const VectorXd& values, const Joints& joints,
                                             moveit::core::RobotState& state)
{
  for (size_t joint_index = 0; joint_index < joints.size(); ++joint_index)
  {
    state.setJointPositions(joints[joint_index], &values[joint_index]);
  }
  return;
}

void KTOptPlanningContext::setJointVelocities(const VectorXd& values, const Joints& joints,
                                              moveit::core::RobotState& state)
{
  for (size_t joint_index = 0; joint_index < joints.size(); ++joint_index)
  {
    state.setJointVelocities(joints[joint_index], &values[joint_index]);
  }
  return;
}

void KTOptPlanningContext::clear()
{
  RCLCPP_ERROR(getLogger(), "KTOptPlanningContext::clear() is not implemented!");
  // do something
}
}  // namespace ktopt_interface
