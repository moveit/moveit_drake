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

double COLLISION_CHECK_RESOLUTION = 25.0;
double LOWER_BOUND = 0.01;
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

  // dome drake related scope initialisations
  const auto& plant = 
    dynamic_cast<const MultibodyPlant<double>&>(diagram_->GetSubsystemByName("plant"));
  const auto& scene_graph =
    dynamic_cast<const SceneGraph<double>&>(diagram_->GetSubsystemByName("scene_graph"));


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
  auto& plant_context = diagram_->GetMutableSubsystemContext(plant, diagram_context_.get());
  plant.SetPositionsAndVelocities(&plant_context, q);

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
  auto trajopt = KinematicTrajectoryOptimization(plant.num_positions(), params_.control_points);
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
  // TODO: Add constraints on joint position/velocity/acceleration
  // trajopt.AddPositionBounds(
  //     plant_->GetPositionLowerLimits(),
  //     plant_->GetPositionUpperLimits());
  // trajopt.AddVelocityBounds(
  //     plant_->GetVelocityLowerLimits(),
  //     plant_->GetVelocityUpperLimits());

  // Add constraints on duration
  trajopt.AddDurationConstraint(0.5, 5);

  // solve the program
  auto result = Solve(prog);

  if (!result.is_success())
  {
    RCLCPP_ERROR(getLogger(), "Trajectory optimization failed");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }

  RCLCPP_INFO(getLogger(), "Setting initial guess ...");
  // set the initial guess
  trajopt.SetInitialGuess(trajopt.ReconstructTrajectory(result));

  // add collision constraints
  for (double s = 0.0; s <=25.0; s++)
  {
    trajopt.AddPathPositionConstraint(
      std::make_shared<MinimumDistanceLowerBoundConstraint>(
        &plant,
        LOWER_BOUND,
        &plant_context),
      s/COLLISION_CHECK_RESOLUTION);
  }
  
  // The previous solution is used to warm-start the collision checked
  // optimization problem
  auto collision_free_result = Solve(prog);

  // package up the resulting trajectory
  auto traj = trajopt.ReconstructTrajectory(collision_free_result);
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

    plant.SetPositions(&plant_context, pos_val);
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
  builder = std::make_unique<DiagramBuilder<double>>();

  // meshcat experiment
  const auto meshcat_params = MeshcatParams();
  meshcat_ = std::make_shared<Meshcat>(meshcat_params);

  auto [plant, scene_graph] = AddMultibodyPlantSceneGraph(builder.get(), 0.0);

  // TODO:(kamiradi) Figure out object parsing
  // auto robot_instance = Parser(plant_, scene_graph_).AddModelsFromString(robot_description_, ".urdf");

  // HACK: For now loading directly from drake's package map
  const char* ModelUrl = "package://drake_models/franka_description/"
                         "urdf/panda_arm.urdf";
  const std::string urdf = PackageMap{}.ResolveUrl(ModelUrl);
  auto robot_instance = Parser(&plant, &scene_graph).AddModels(urdf);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));

  // planning scene transcription
  const auto scene = getPlanningScene();
  transcribePlanningScene(*scene);


  // for now finalize plant here
  plant.Finalize();

  // Apply MeshCat visualization
  VisualizationConfig config;
  ApplyVisualizationConfig(config, builder.get(), /*lcm_buses*/ nullptr, &plant, &scene_graph, meshcat_);

  MeshcatVisualizerParams meshcat_viz_params;
  auto& visualizer =
      MeshcatVisualizer<double>::AddToBuilder(builder.get(), scene_graph, meshcat_,
                                              std::move(meshcat_viz_params));
  visualizer_ = &visualizer;

  // in the future you can add other LeafSystems here. For now building the
  // diagram
  diagram_ = builder->Build();
  diagram_context_ = diagram_->CreateDefaultContext();
  auto& plant_context = diagram_->GetMutableSubsystemContext(plant, diagram_context_.get());

  nominal_q_ = plant.GetPositions(plant_context);

  auto& vis_context = visualizer_->GetMyContextFromRoot(*diagram_context_);
  visualizer_->ForcedPublish(vis_context);
}

void KTOptPlanningContext::transcribePlanningScene(const planning_scene::PlanningScene& planning_scene)
{
  try
  {
    auto world = planning_scene.getWorld();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(getLogger(), "caught exception ... " << e.what());
  }
  auto& plant =
    dynamic_cast<MultibodyPlant<double>&>(builder->GetMutableSubsystemByName("plant"));
  auto& scene_graph =
    dynamic_cast<SceneGraph<double>&>(builder->GetMutableSubsystemByName("scene_graph"));
  for (const auto& object : planning_scene.getWorld()->getObjectIds())
  {
    const auto& collision_object = planning_scene.getWorld()->getObject(object);
    if (!collision_object)
    {
      RCLCPP_INFO(getLogger(), "No collision object");
      return;
    }
    if (object == OCTOMAP_NS)
    {
      RCLCPP_ERROR(getLogger(), "skipping octomap for now ...");
      continue;
    }
    for (const auto& shape : collision_object->shapes_)
    {
      RCLCPP_ERROR(getLogger(), "iterating inside collision object's shapes");
      const auto& pose = collision_object->shape_poses_[0];

      // Creates a box geometry and anchors it to the world origin. Better
      // approach is to create a ground object, anchor that, and then anchor
      // every non-moving entity to the ground plane
      // TODO: Create and anchor ground entity
      Vector3d p(0.3, -0.3, 0.5);
      const SourceId box_source_id = scene_graph.RegisterSource("box1");
      const GeometryId box_geom_id = scene_graph.RegisterAnchoredGeometry(
        box_source_id,
        std::make_unique<GeometryInstance>(
          RigidTransformd(p),
          std::make_unique<Box>(
            0.15,
            0.15,
            0.15),
          "box"
        )); //hard coded for now because I know box dimensions and pose, from

      // add illustration, proximity, perception properties
      scene_graph.AssignRole(box_source_id, box_geom_id, IllustrationProperties());
      scene_graph.AssignRole(box_source_id, box_geom_id, ProximityProperties());
      scene_graph.AssignRole(box_source_id, box_geom_id, PerceptionProperties());
    }
  }
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
