#include <memory>
#include <moveit/planning_interface/planning_interface.hpp>
#include <moveit/planning_interface/planning_response.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/utils/logger.hpp>
#include <class_loader/class_loader.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>
#include <ktopt_interface/ktopt_planning_context.hpp>

namespace ktopt_interface
{
using std::placeholders::_1;
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.ktopt.planner_manager");
}

/**
 * @brief Implementation for the Drake Kinematic Trajectory Optimization (KTOpt) motion planner in MoveIt.
 */
class KTOptPlannerManager : public planning_interface::PlannerManager
{
public:
  KTOptPlannerManager() = default;
  ~KTOptPlannerManager() override = default;

  bool initialize(const moveit::core::RobotModelConstPtr& model, const rclcpp::Node::SharedPtr& node,
                  const std::string& parameter_namespace) override
  {
    robot_model_ = model;
    node_ = node;
    param_listener_ = std::make_shared<ktopt_interface::ParamListener>(node, parameter_namespace);

    // set QoS to transient local to get messages that have already been published
    // (if robot state publisher starts before planner)
    robot_description_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
        "robot_description", rclcpp::QoS(1).transient_local(), [this](const std_msgs::msg::String::SharedPtr msg) {
          if (robot_description_.empty())
          {
            robot_description_ = msg->data;
            RCLCPP_INFO(getLogger(), "Robot description set");
          }
        });
    RCLCPP_INFO(getLogger(), "KTOpt planner manager initialized!");
    return true;
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override
  {
    if (robot_description_.empty())
    {
      RCLCPP_ERROR(getLogger(), "Robot description is empty, do you have a robot state publisher running?");
      return false;
    }
    if (req.goal_constraints.empty())
    {
      RCLCPP_ERROR(getLogger(), "Invalid goal constraints");
      return false;
    }

    if (req.group_name.empty() || !robot_model_->hasJointModelGroup(req.group_name))
    {
      RCLCPP_ERROR(getLogger(), "Invalid joint group '%s'", req.group_name.c_str());
      return false;
    }

    return true;
  }

  std::string getDescription() const override
  {
    return "KTOpt";
  }

  void getPlanningAlgorithms(std::vector<std::string>& algs) const override
  {
    algs.clear();
    algs.push_back("ktopt");
  }

  planning_interface::PlanningContextPtr
  getPlanningContext(const planning_scene::PlanningSceneConstPtr& planning_scene,
                     const planning_interface::MotionPlanRequest& req,
                     moveit_msgs::msg::MoveItErrorCodes& error_code) const override
  {
    if (!canServiceRequest(req))
    {
      error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
      return nullptr;
    }

    const auto params = param_listener_->get_params();
    std::shared_ptr<KTOptPlanningContext> planning_context =
        std::make_shared<KTOptPlanningContext>("KTOPT", req.group_name, params);
    // set robot description
    planning_context->setPlanningScene(planning_scene);
    planning_context->setRobotDescription(robot_description_);
    planning_context->setMotionPlanRequest(req);

    return planning_context;
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ktopt_interface::ParamListener> param_listener_;

  // robot description related variables
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  bool description_set_;
  std::string robot_description_;
};

}  // namespace ktopt_interface

// register the KTOptPlannerManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(ktopt_interface::KTOptPlannerManager, planning_interface::PlannerManager);
