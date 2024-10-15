#include <cmath>

#include <drake/geometry/meshcat_params.h>
#include <drake/geometry/geometry_frame.h>
#include <drake/geometry/geometry_instance.h>
#include <drake/geometry/geometry_roles.h>
#include <drake/geometry/proximity_properties.h>
#include <drake/solvers/solve.h>
#include <drake/visualization/visualization_config.h>
#include <drake/visualization/visualization_config_functions.h>

#include <moveit/constraint_samplers/constraint_sampler_manager.h>
#include <moveit/drake/conversions.hpp>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/conversions.h>

#include <ktopt_interface/ktopt_planning_context.hpp>

namespace ktopt_interface
{
namespace
{
/// @brief Helper function that returns the logger instance associated with this planner.
/// @return The logger instance.
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.ktopt_interface.planning_context");
}

/// @brief The namespace corresponding to the octomap in the planning scene.
constexpr auto kOctomapNamespace = "<octomap>";
}  // namespace

KTOptPlanningContext::KTOptPlanningContext(const std::string& name, const std::string& group_name,
                                           const ktopt_interface::Params& params)
  : planning_interface::PlanningContext(name, group_name), params_(params)
{
  // Do some drake initialization may be
}

void KTOptPlanningContext::solve(planning_interface::MotionPlanDetailedResponse& /*res*/)
{
  RCLCPP_ERROR(getLogger(), "KTOptPlanningContext::solve(planning_interface::"
                            "MotionPlanDetailedResponse&) is not implemented!");
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

  // Get velocity and acceleration bounds
  Eigen::VectorXd lower_position_bounds;
  Eigen::VectorXd upper_position_bounds;
  Eigen::VectorXd lower_velocity_bounds;
  Eigen::VectorXd upper_velocity_bounds;
  Eigen::VectorXd lower_acceleration_bounds;
  Eigen::VectorXd upper_acceleration_bounds;
  Eigen::VectorXd lower_jerk_bounds;
  Eigen::VectorXd upper_jerk_bounds;

  moveit::drake::getPositionBounds(joint_model_group, plant, lower_position_bounds, upper_position_bounds);
  moveit::drake::getVelocityBounds(joint_model_group, plant, lower_velocity_bounds, upper_velocity_bounds);
  moveit::drake::getAccelerationBounds(joint_model_group, plant, lower_acceleration_bounds, upper_acceleration_bounds);
  moveit::drake::getJerkBounds(joint_model_group, plant, lower_jerk_bounds, upper_jerk_bounds);

  // q represents the complete state (joint positions and velocities)
  auto q = Eigen::VectorXd::Zero(plant.num_positions() + plant.num_velocities());
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

  // Add costs
  trajopt.AddDurationCost(params_.duration_cost_weight);
  trajopt.AddPathLengthCost(params_.path_length_cost_weight);
  // TODO: Adds quadratic cost
  // This acts as a secondary cost to push the solution towards joint centers
  // prog.AddQuadraticErrorCost(Eigen::MatrixXd::Identity(plant.num_positions(), plant.num_positions()), nominal_q_,
  // prog.control_points());

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

  // Add constraints on joint kinematic limits.
  trajopt.AddPositionBounds(lower_position_bounds, upper_position_bounds);
  trajopt.AddVelocityBounds(lower_velocity_bounds, upper_velocity_bounds);
  trajopt.AddAccelerationBounds(lower_acceleration_bounds, upper_acceleration_bounds);
  trajopt.AddJerkBounds(lower_jerk_bounds, upper_jerk_bounds);

  // Add constraints on duration
  if (params_.min_trajectory_time > params_.max_trajectory_time)
  {
    RCLCPP_ERROR(getLogger(), "Minimum trajectory time cannot be greater than maximum trajectory time.");
    res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
    return;
  }
  trajopt.AddDurationConstraint(params_.min_trajectory_time, params_.max_trajectory_time);

  // process path_constraints
  addPathPositionConstraints(trajopt, plant, plant_context, params_.position_constraint_padding);

  // solve the program
  auto result = drake::solvers::Solve(prog);

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

  // The previous solution is used to warm-start the collision checked
  // optimization problem
  auto collision_free_result = Solve(prog);

  // package up the resulting trajectory
  auto traj = trajopt.ReconstructTrajectory(collision_free_result);
  res.trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(start_state.getRobotModel(), joint_model_group);

  moveit::drake::getRobotTrajectory(traj, params_.trajectory_time_step, plant, res.trajectory);

  // Visualize the trajectory with Meshcat

  // TODO: add to conversions
  if (params_.meshcat_visualise)
  {
    visualizer_->StartRecording();
    const auto num_pts = static_cast<size_t>(std::ceil(traj.end_time() / params_.trajectory_time_step) + 1);
    for (unsigned int i = 0; i < num_pts; ++i)
    {
      const auto t_scale = static_cast<double>(i) / static_cast<double>(num_pts - 1);
      const auto t = std::min(t_scale, 1.0) * traj.end_time();
      plant.SetPositions(&plant_context, traj.value(t));
      auto& vis_context = visualizer_->GetMyContextFromRoot(*diagram_context_);
      visualizer_->ForcedPublish(vis_context);
      // Without these sleeps, the visualizer won't give you time to load your
      // browser
      // TODO: This should not hold up planning time
      std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(params_.trajectory_time_step * 10000.0)));
    }
    visualizer_->StopRecording();
    visualizer_->PublishRecording();
  }
  res.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return;
}

void KTOptPlanningContext::addPathPositionConstraints(KinematicTrajectoryOptimization& trajopt,
                                                      const MultibodyPlant<double>& plant,
                                                      Context<double>& plant_context, const double padding)
{
  // retrieve the motion planning request
  const auto& req = getMotionPlanRequest();

  // check for path position constraints
  if (!req.path_constraints.position_constraints.empty())
  {
    for (const auto& position_constraint : req.path_constraints.position_constraints)
    {
      // Extract the bounding box's center (primitive pose)
      const auto& primitive_pose = position_constraint.constraint_region.primitive_poses[0];
      Eigen::Vector3d box_center(primitive_pose.position.x, primitive_pose.position.y, primitive_pose.position.z);

      // Extract dimensions of the bounding box from
      // constraint_region.primitives Assuming it is a box
      // (shape_msgs::SolidPrimitive::BOX) and has dimensions in [x, y, z]
      const auto link_ee_name = position_constraint.link_name;
      if (!plant.HasBodyNamed(link_ee_name))
      {
        RCLCPP_ERROR(getLogger(),
                     "The link specified in the PositionConstraint message, %s, does not exist in the Drake "
                     "plant.",
                     link_ee_name.c_str());
        continue;
      }
      const auto& link_ee_frame = plant.GetFrameByName(link_ee_name);

      const auto base_frame_name = position_constraint.header.frame_id;
      if (!plant.HasBodyNamed(base_frame_name))
      {
        RCLCPP_ERROR(getLogger(),
                     "The link specified in the PositionConstraint message, %s, does not exist in the Drake "
                     "plant.",
                     base_frame_name.c_str());
        continue;
      }
      const auto& base_frame = plant.GetFrameByName(base_frame_name);

      const auto& primitive = position_constraint.constraint_region.primitives[0];
      if (primitive.type != shape_msgs::msg::SolidPrimitive::BOX)
      {
        RCLCPP_WARN(getLogger(), "Expects a bounding box constraint as a SolidPrimitive::BOX");
        continue;
      }

      // Calculate the lower and upper bounds based on the box dimensions
      // around the center
      const auto offset_vec = Eigen::Vector3d(std::max(0.0, primitive.dimensions[0] / 2.0 - padding),
                                              std::max(0.0, primitive.dimensions[1] / 2.0 - padding),
                                              std::max(0.0, primitive.dimensions[2] / 2.0 - padding));
      const auto lower_bound = box_center - offset_vec;
      const auto upper_bound = box_center + offset_vec;

      // Add position constraint to each knot point of the trajectory
      for (int i = 0; i < params_.num_position_constraint_points; ++i)
      {
        trajopt.AddPathPositionConstraint(
            std::make_shared<PositionConstraint>(&plant, base_frame, lower_bound, upper_bound, link_ee_frame,
                                                 Eigen::Vector3d(0.0, 0.0, 0.0), &plant_context),
            static_cast<double>(i) / (params_.num_position_constraint_points - 1));
      }
    }
  }
}

bool KTOptPlanningContext::terminate()
{
  RCLCPP_ERROR(getLogger(), "KTOptPlanningContext::terminate() is not implemented!");
  return true;
}

void KTOptPlanningContext::setRobotDescription(const std::string& robot_description)
{
  robot_description_ = robot_description;

  // also perform some drake related initialisations here
  builder = std::make_unique<DiagramBuilder<double>>();

  // meshcat experiment
  const auto meshcat_params = drake::geometry::MeshcatParams();
  meshcat_ = std::make_shared<drake::geometry::Meshcat>(meshcat_params);

  auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(builder.get(), 0.0);

  // TODO:(kamiradi) Figure out object parsing
  // auto robot_instance = Parser(plant_,
  // scene_graph_).AddModelsFromString(robot_description_, ".urdf");

  const char* ModelUrl = params_.drake_robot_description.c_str();
  const std::string urdf = drake::multibody::PackageMap{}.ResolveUrl(ModelUrl);
  auto robot_instance = drake::multibody::Parser(&plant, &scene_graph).AddModels(urdf);
  plant.WeldFrames(plant.world_frame(), plant.GetFrameByName("panda_link0"));

  // planning scene transcription
  const auto scene = getPlanningScene();
  transcribePlanningScene(*scene);

  // for now finalize plant here
  plant.Finalize();

  // Apply MeshCat visualization
  drake::visualization::VisualizationConfig config;
  drake::visualization::ApplyVisualizationConfig(config, builder.get(), /*lcm_buses*/ nullptr, &plant, &scene_graph,
                                                 meshcat_);

  drake::geometry::MeshcatVisualizerParams meshcat_viz_params;
  auto& visualizer = drake::geometry::MeshcatVisualizer<double>::AddToBuilder(builder.get(), scene_graph, meshcat_,
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
  // Transcribe the planning scene into a drake scene graph
  try
  {
    auto world = planning_scene.getWorld();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_STREAM(getLogger(), "caught exception ... " << e.what());
  }
  auto& scene_graph = builder->GetMutableDowncastSubsystemByName<drake::geometry::SceneGraph<double>>("scene_graph");
  for (const auto& object : planning_scene.getWorld()->getObjectIds())
  {
    const auto& collision_object = planning_scene.getWorld()->getObject(object);
    if (!collision_object)
    {
      RCLCPP_INFO(getLogger(), "No collision object");
      return;
    }
    if (object == kOctomapNamespace)
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

      std::unique_ptr<drake::geometry::Shape> shape_ptr;
      switch (shape_type)
      {
        case shapes::ShapeType::BOX:
        {
          const auto object_ptr = std::dynamic_pointer_cast<const shapes::Box>(shape);
          shape_ptr =
              std::make_unique<drake::geometry::Box>(object_ptr->size[0], object_ptr->size[1], object_ptr->size[2]);
          break;
        }
        case shapes::ShapeType::SPHERE:
        {
          const auto object_ptr = std::dynamic_pointer_cast<const shapes::Sphere>(shape);
          shape_ptr = std::make_unique<drake::geometry::Sphere>(object_ptr->radius);
          break;
        }
        case shapes::ShapeType::CYLINDER:
        {
          const auto object_ptr = std::dynamic_pointer_cast<const shapes::Cylinder>(shape);
          shape_ptr = std::make_unique<drake::geometry::Cylinder>(object_ptr->radius, object_ptr->length);
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
      const auto source_id = scene_graph.RegisterSource(shape_name);
      const auto geom_id = scene_graph.RegisterAnchoredGeometry(
          source_id, std::make_unique<drake::geometry::GeometryInstance>(drake::math::RigidTransformd(pose),
                                                                         std::move(shape_ptr), shape_name));

      // add illustration, proximity, perception properties
      scene_graph.AssignRole(source_id, geom_id, drake::geometry::IllustrationProperties());
      scene_graph.AssignRole(source_id, geom_id, drake::geometry::ProximityProperties());
      scene_graph.AssignRole(source_id, geom_id, drake::geometry::PerceptionProperties());

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
