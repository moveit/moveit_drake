#include <memory>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/logger.hpp>
#include <class_loader/class_loader.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <std_msgs/msg/string.hpp>
#include <ktopt_interface/ktopt_planning_context.hpp>

// just so that this builds

namespace ktopt_interface
{
using std::placeholders::_1;
rclcpp::Logger getLogger()
{
  return moveit::getLogger("moveit.planners.ktopt.planner_manager");
}
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
    parameter_namespace_ = parameter_namespace;
    param_listener_ = std::make_shared<ktopt_interface::ParamListener>(node, parameter_namespace);
    //robot_description_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
    //    "robot_description", 10, std::bind(&KTOptPlannerManager::robot_description_callback, this, _1));
    robot_description_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::String::SharedPtr msg){
        if (!description_set_)
        {
          robot_description_ = msg->data;
          description_set_ = true;
          RCLCPP_INFO(getLogger(), "Robot description set");
        }
      });
    description_set_ = false;
    RCLCPP_INFO(getLogger(), "KTOpt planner manager initialized!");
    return true;
  }

  bool canServiceRequest(const planning_interface::MotionPlanRequest& req) const override
  {
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

    // if (!description_set_)
    // {
    //   RCLCPP_ERROR_STREAM(getLogger(), "Robot description has not yet been set, " 
    //                << "this means that Drake's internal MultibodyPlant "
    //                << "and SceneGraph have not been initialised.");
    //   // DEBG: Created a separate function to re-subscribe to the
    //   // robot_description topic, but cannot execute it as this funciton itself
    //   // is a const function, cannot modify the object from which it is called.
    //   // Any function that is called inside as well will not be able to change
    //   // the state of the object.

    //   // initializeRobotDescription();
    //   return false;
    // }
    return true;
  }

  void initializeRobotDescription()
  {
    RCLCPP_INFO_STREAM(getLogger(), "Reinitializing subscription to robot description");
    robot_description_subscriber_ = node_->create_subscription<std_msgs::msg::String>(
      "/robot_description", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::String::SharedPtr msg)
      {
        if (!description_set_)
        {
          robot_description_ = msg->data;
          description_set_ = true;
          RCLCPP_INFO(getLogger(), "Robot description set");
        }
      });

    return;
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
    planning_context->setRobotDescription(robot_description_);
    planning_context->setPlanningScene(planning_scene);
    planning_context->setMotionPlanRequest(req);

    return planning_context;
  }

private:
  moveit::core::RobotModelConstPtr robot_model_;
  rclcpp::Node::SharedPtr node_;
  std::string parameter_namespace_;
  std::shared_ptr<ktopt_interface::ParamListener> param_listener_;

  // robot description related variables
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_subscriber_;
  bool description_set_;
  std::string robot_description_;

  void robot_description_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    RCLCPP_INFO(getLogger(), "Retrieved robot description!!");
    if (!description_set_)
    {
      robot_description_ = msg->data;
      description_set_ = true;
      RCLCPP_INFO_STREAM(getLogger(), "Robot description has been set.");
    }
  }
};

}  // namespace ktopt_interface

// register the KTOptPlannerManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(ktopt_interface::KTOptPlannerManager, planning_interface::PlannerManager);
