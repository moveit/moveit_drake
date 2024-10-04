#include <cmath>

#include <moveit/drake/conversions.hpp>
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

constexpr double kDefaultJointMaxPosition = 1.0e6;

}  // namespace

KTOptPlanningContext::KTOptPlanningContext(const std::string& name, const std::string& group_name,
                                           const ktopt_interface::Params& params)
  : planning_interface::PlanningContext(name, group_name), params_(params)
{
  // Do some drake initialization may be
}

void KTOptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& /*res*/)
{
  RCLCPP_ERROR(getLogger(),
               "KTOptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse&) is not implemented!");
  return;
}

void KTOptPlanningContext::solve(planning_interface::MotionPlanResponse& res)
{
  RCLCPP_INFO(getLogger(), "Setting up and solving optimization problem ...");
  // preliminary house keeping
  res.planner_id = std::string("ktopt");
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;

  // some drake related scope initialisations
  const auto& plant = diagram_->GetDowncastSubsystemByName<MultibodyPlant<double>>("plant");

  // Retrieve motion plan request
  const auto& req = getMotionPlanRequest();
  const moveit::core::RobotState start_state(*getPlanningScene()->getCurrentStateUpdated(req.start_state));
  const auto joint_model_group = getPlanningScene()->getRobotModel()->getJointModelGroup(getGroupName());
  RCLCPP_INFO_STREAM(getLogger(), "Planning for group: " << getGroupName());
  const int num_positions = plant.num_positions();
  const int num_velocities = plant.num_velocities();

  // extract position and velocity bounds
  std::vector<double> lower_position_bounds;
  std::vector<double> upper_position_bounds;
  std::vector<double> lower_velocity_bounds;
  std::vector<double> upper_velocity_bounds;
  std::vector<double> lower_acceleration_bounds;
  std::vector<double> upper_acceleration_bounds;
  lower_position_bounds.reserve(num_positions);
  upper_position_bounds.reserve(num_positions);
  lower_velocity_bounds.reserve(num_positions);
  upper_velocity_bounds.reserve(num_positions);
  lower_acceleration_bounds.reserve(num_positions);
  upper_acceleration_bounds.reserve(num_positions);

  for (const auto& joint_model : joint_model_group->getActiveJointModels())
  {
    const auto& joint_name = joint_model->getName();
    const auto& bounds = joint_model->getVariableBounds()[0];

    lower_position_bounds.push_back(bounds.min_position_);
    upper_position_bounds.push_back(bounds.max_position_);
    lower_velocity_bounds.push_back(-bounds.max_velocity_);
    upper_velocity_bounds.push_back(bounds.max_velocity_);
    lower_acceleration_bounds.push_back(-bounds.max_acceleration_);
    upper_acceleration_bounds.push_back(bounds.max_acceleration_);
  }

  // Ensure the bounds have the correct size
  if (lower_position_bounds.size() != num_positions || upper_position_bounds.size() != num_positions)
  {
    lower_position_bounds.resize(num_positions, -kDefaultJointMaxPosition);
    upper_position_bounds.resize(num_positions, kDefaultJointMaxPosition);
  }
  if (lower_velocity_bounds.size() != num_velocities || upper_velocity_bounds.size() != num_velocities)
  {
    // Resize velocity bounds to match number of velocities, if necessary
    lower_velocity_bounds.resize(num_velocities, -kDefaultJointMaxPosition);
    upper_velocity_bounds.resize(num_velocities, kDefaultJointMaxPosition);
  }
  if (lower_acceleration_bounds.size() != num_velocities || upper_acceleration_bounds.size() != num_velocities)
  {
    // Resize velocity bounds to match number of velocities, if necessary
    lower_acceleration_bounds.resize(num_velocities, -kDefaultJointMaxPosition);
    upper_acceleration_bounds.resize(num_velocities, kDefaultJointMaxPosition);
  }
  Eigen::Map<const Eigen::VectorXd> lower_position_bounds_eigen(lower_position_bounds.data(),
                                                                lower_position_bounds.size());
  Eigen::Map<const Eigen::VectorXd> upper_position_bounds_eigen(upper_position_bounds.data(),
                                                                upper_position_bounds.size());
  Eigen::Map<const Eigen::VectorXd> lower_velocity_bounds_eigen(lower_velocity_bounds.data(),
                                                                lower_velocity_bounds.size());
  Eigen::Map<const Eigen::VectorXd> upper_velocity_bounds_eigen(upper_velocity_bounds.data(),
                                                                upper_velocity_bounds.size());
  Eigen::Map<const Eigen::VectorXd> lower_acceleration_bounds_eigen(lower_acceleration_bounds.data(),
                                                                    lower_acceleration_bounds.size());
  Eigen::Map<const Eigen::VectorXd> upper_acceleration_bounds_eigen(upper_acceleration_bounds.data(),
                                                                    upper_acceleration_bounds.size());

  Eigen::VectorXd lower_jerk_bounds_eigen = Eigen::VectorXd::Constant(num_velocities, -1 * params_.joint_jerk_bound);
  Eigen::VectorXd upper_jerk_bounds_eigen = Eigen::VectorXd::Constant(num_velocities, params_.joint_jerk_bound);

  // q represents the complete state (joint positions and velocities)
  VectorXd q = VectorXd::Zero(plant.num_positions() + plant.num_velocities());
  q << moveit::drake::getJointPositionVector(start_state, getGroupName(), plant);
  q << moveit::drake::getJointVelocityVector(start_state, getGroupName(), plant);

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
  auto trajopt = KinematicTrajectoryOptimization(plant.num_positions(), params_.num_control_points);
  auto& prog = trajopt.get_mutable_prog();

  // Costs
  // TODO: These should be parameters
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
  const auto& start_position = moveit::drake::getJointPositionVector(start_state, getGroupName(), plant);
  trajopt.AddPathPositionConstraint(start_position, start_position, 0.0);
  const auto& start_velocity = moveit::drake::getJointVelocityVector(start_state, getGroupName(), plant);
  trajopt.AddPathVelocityConstraint(start_velocity, start_velocity, 0.0);
  // Add constraint on end joint configuration and velocity
  const auto& goal_position = moveit::drake::getJointPositionVector(goal_state, getGroupName(), plant);
  trajopt.AddPathPositionConstraint(goal_position, goal_position, 1.0);
  const auto& goal_velocity = moveit::drake::getJointVelocityVector(goal_state, getGroupName(), plant);
  trajopt.AddPathVelocityConstraint(goal_velocity, goal_velocity, 1.0);

  // TODO: Add constraints on joint position/velocity/acceleration
  trajopt.AddPositionBounds(lower_position_bounds_eigen, upper_position_bounds_eigen);
  trajopt.AddVelocityBounds(lower_velocity_bounds_eigen, upper_velocity_bounds_eigen);
  trajopt.AddAccelerationBounds(lower_acceleration_bounds_eigen, upper_acceleration_bounds_eigen);
  trajopt.AddJerkBounds(lower_jerk_bounds_eigen, upper_jerk_bounds_eigen);

  // Add constraints on duration
  // TODO: These should be parameters
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
  for (int s = 0; s < params_.num_collision_check_points; ++s)
  {
    trajopt.AddPathPositionConstraint(std::make_shared<MinimumDistanceLowerBoundConstraint>(
                                          &plant, params_.collision_check_lower_distance_bound, &plant_context),
                                      static_cast<double>(s) / (params_.num_collision_check_points - 1));
  }

  // The previous solution is used to warm-start the collision checked optimization problem
  auto collision_free_result = Solve(prog);

  // package up the resulting trajectory
  auto traj = trajopt.ReconstructTrajectory(collision_free_result);
  res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(start_state.getRobotModel(), joint_model_group);

  moveit::drake::getRobotTrajectory(traj, params_.trajectory_time_step, res.trajectory);

  // Visualize the trajectory with Meshcat
  visualizer_->StartRecording();
  const auto num_pts = static_cast<size_t>(std::ceil(traj.end_time() / params_.trajectory_time_step) + 1);
  for (unsigned int i = 0; i < num_pts; ++i)
  {
    const auto t_scale = static_cast<double>(i) / static_cast<double>(num_pts - 1);
    const auto t = std::min(t_scale, 1.0) * traj.end_time();
    plant.SetPositions(&plant_context, traj.value(t));
    auto& vis_context = visualizer_->GetMyContextFromRoot(*diagram_context_);
    visualizer_->ForcedPublish(vis_context);
    // Without these sleeps, the visualizer won't give you time to load your browser
    // TODO: This should not hold up planning time
    std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(params_.trajectory_time_step * 10000.0)));
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

  const char* ModelUrl = params_.drake_robot_description.c_str();
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
      MeshcatVisualizer<double>::AddToBuilder(builder.get(), scene_graph, meshcat_, std::move(meshcat_viz_params));
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
  // Transcribe the planning scene into a drake scene graph
  try
  {
    auto world = planning_scene.getWorld();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(getLogger(), "caught exception ... " << e.what());
  }
  auto& scene_graph = builder->GetMutableDowncastSubsystemByName<SceneGraph<double>>("scene_graph");
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
      RCLCPP_WARN(getLogger(), "Octomap not supported for now ... ");
      continue;
    }
    for (size_t i = 0; i < collision_object->shapes_.size(); ++i)
    {
      const std::string shape_name = object + std::to_string(i);
      const auto& shape = collision_object->shapes_[i];
      const auto& pose = collision_object->pose_;
      const auto& shape_type = collision_object->shapes_[i]->type;

      std::unique_ptr<Shape> shape_ptr;
      switch (shape_type)
      {
        case shapes::ShapeType::BOX:
        {
          const auto object_ptr = std::dynamic_pointer_cast<const shapes::Box>(shape);
          shape_ptr = std::make_unique<Box>(object_ptr->size[0], object_ptr->size[1], object_ptr->size[2]);
          break;
        }
        case shapes::ShapeType::SPHERE:
        {
          const auto object_ptr = std::dynamic_pointer_cast<const shapes::Sphere>(shape);
          shape_ptr = std::make_unique<Sphere>(object_ptr->radius);
          break;
        }
        case shapes::ShapeType::CYLINDER:
        {
          const auto object_ptr = std::dynamic_pointer_cast<const shapes::Cylinder>(shape);
          shape_ptr = std::make_unique<Cylinder>(object_ptr->radius, object_ptr->length);
          break;
        }
        default:
        {
          RCLCPP_WARN(getLogger(), "Unsupported shape for '%s', ignoring in scene graph.", shape_name.c_str());
          break;
        }
      }

      if (!shape_ptr)
      {
        continue;
      }

      // Register the geometry in the scene graph.
      const SourceId source_id = scene_graph.RegisterSource(shape_name);
      const GeometryId geom_id = scene_graph.RegisterAnchoredGeometry(
          source_id, std::make_unique<GeometryInstance>(RigidTransformd(pose), std::move(shape_ptr), shape_name));

      // add illustration, proximity, perception properties
      scene_graph.AssignRole(source_id, geom_id, IllustrationProperties());
      scene_graph.AssignRole(source_id, geom_id, ProximityProperties());
      scene_graph.AssignRole(source_id, geom_id, PerceptionProperties());

      // TODO: Create and anchor ground entity
    }
  }
}

void KTOptPlanningContext::clear()
{
  RCLCPP_ERROR(getLogger(), "KTOptPlanningContext::clear() is not implemented!");
  // do something
}
}  // namespace ktopt_interface
