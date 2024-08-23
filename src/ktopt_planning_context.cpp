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

  // dome drake related scope initialisations
  const auto& plant = dynamic_cast<const MultibodyPlant<double>&>(diagram_->GetSubsystemByName("plant"));

  // Retrieve motion plan request
  const auto& req = getMotionPlanRequest();
  const moveit::core::RobotState start_state(*getPlanningScene()->getCurrentStateUpdated(req.start_state));
  const auto joint_model_group = getPlanningScene()->getRobotModel()->getJointModelGroup(getGroupName());
  RCLCPP_INFO_STREAM(getLogger(), "Planning for group: " << getGroupName());
  const auto& joints = joint_model_group->getActiveJointModels();

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
  trajopt.AddPathPositionConstraint(moveit::drake::getJointPositionVector(start_state, getGroupName(), plant),
                                    moveit::drake::getJointPositionVector(start_state, getGroupName(), plant), 0.0);
  trajopt.AddPathVelocityConstraint(moveit::drake::getJointVelocityVector(start_state, getGroupName(), plant),
                                    moveit::drake::getJointVelocityVector(start_state, getGroupName(), plant), 0.0);
  // Add constraint on end joint configuration and velocity
  trajopt.AddPathPositionConstraint(moveit::drake::getJointPositionVector(goal_state, getGroupName(), plant),
                                    moveit::drake::getJointPositionVector(goal_state, getGroupName(), plant), 1.0);
  trajopt.AddPathVelocityConstraint(moveit::drake::getJointVelocityVector(goal_state, getGroupName(), plant),
                                    moveit::drake::getJointVelocityVector(goal_state, getGroupName(), plant), 1.0);

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
  double t_prev = 0.0;
  const auto num_pts = static_cast<size_t>(std::ceil(traj.end_time() / params_.trajectory_time_step) + 1);
  for (unsigned int i = 0; i < num_pts; ++i)
  {
    const auto t_scale = static_cast<double>(i) / static_cast<double>(num_pts - 1);
    const auto t = std::min(t_scale, 1.0) * traj.end_time();
    plant.SetPositions(&plant_context, traj.value(t));
    auto& vis_context = visualizer_->GetMyContextFromRoot(*diagram_context_);
    visualizer_->ForcedPublish(vis_context);
    t_prev = t;
    // Without these sleeps, the visualizer won't give you time to load your browser
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

  // HACK: For now loading directly from drake's package map
  const char* ModelUrl = "package://drake_models/franka_description/"
                         "urdf/panda_arm_hand.urdf";
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
  auto& scene_graph = dynamic_cast<SceneGraph<double>&>(builder->GetMutableSubsystemByName("scene_graph"));
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
      std::string shape_name = object + std::to_string(i);
      const auto& shape = collision_object->shapes_[i];
      const auto& pose = collision_object->pose_;
      const auto& shape_type = collision_object->shapes_[i]->type;

      switch (shape_type)
      {
        case shapes::ShapeType::BOX:
        {
          const auto objectptr = std::dynamic_pointer_cast<const shapes::Box>(shape);
          const SourceId box_source_id = scene_graph.RegisterSource(shape_name);
          const GeometryId box_geom_id = scene_graph.RegisterAnchoredGeometry(
              box_source_id,
              std::make_unique<GeometryInstance>(
                  RigidTransformd(pose),
                  std::make_unique<Box>(objectptr->size[0], objectptr->size[1], objectptr->size[2]), shape_name));

          // add illustration, proximity, perception properties
          scene_graph.AssignRole(box_source_id, box_geom_id, IllustrationProperties());
          scene_graph.AssignRole(box_source_id, box_geom_id, ProximityProperties());
          scene_graph.AssignRole(box_source_id, box_geom_id, PerceptionProperties());
          break;
        }
        case shapes::ShapeType::UNKNOWN_SHAPE:
        {
          RCLCPP_WARN(getLogger(), "Unknown shape, ignoring in scene graph");
          break;
        }
        case shapes::ShapeType::SPHERE:
        {
          const auto objectptr = std::dynamic_pointer_cast<const shapes::Sphere>(shape);
          const SourceId box_source_id = scene_graph.RegisterSource(shape_name);
          const GeometryId box_geom_id = scene_graph.RegisterAnchoredGeometry(
              box_source_id, std::make_unique<GeometryInstance>(
                                 RigidTransformd(pose), std::make_unique<Sphere>(objectptr->radius), shape_name));
          RCLCPP_INFO(getLogger(), "Sphere");

          // add illustration, proximity, perception properties
          scene_graph.AssignRole(box_source_id, box_geom_id, IllustrationProperties());
          scene_graph.AssignRole(box_source_id, box_geom_id, ProximityProperties());
          scene_graph.AssignRole(box_source_id, box_geom_id, PerceptionProperties());
          break;
        }
        case shapes::ShapeType::CYLINDER:
        {
          const auto objectptr = std::dynamic_pointer_cast<const shapes::Cylinder>(shape);
          const SourceId box_source_id = scene_graph.RegisterSource(shape_name);
          const GeometryId box_geom_id = scene_graph.RegisterAnchoredGeometry(
              box_source_id,
              std::make_unique<GeometryInstance>(
                  RigidTransformd(pose), std::make_unique<Cylinder>(objectptr->radius, objectptr->length), shape_name));
          RCLCPP_INFO(getLogger(), "Cylinder");

          // add illustration, proximity, perception properties
          scene_graph.AssignRole(box_source_id, box_geom_id, IllustrationProperties());
          scene_graph.AssignRole(box_source_id, box_geom_id, ProximityProperties());
          scene_graph.AssignRole(box_source_id, box_geom_id, PerceptionProperties());
          break;
        }
        case shapes::ShapeType::CONE:
        {
          RCLCPP_WARN(getLogger(), "Cone not supported in drake");
          break;
        }
        default:
        {
          RCLCPP_WARN(getLogger(), "Shape TYPE conversation to drake is not implemented");
        }
      }

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
