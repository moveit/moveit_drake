#include <memory>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/utils/logger.hpp>
#include <class_loader/class_loader.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/logging.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ktopt_interface/ktopt_planning_context.h>

// just so that this builds

namespace ktopt_interface
{
rclcpp::Logger getLogger()
{
    return moveit::getLogger("moveit.planners.ktopt.planner_manager");
}
class KTOptPlannerManager : public planning_interface::PlannerManager
{
public:
    KTOptPlannerManager() = default; 
    ~KTOptPlannerManager() override = default;

    bool initialize(
        const moveit::core::RobotModelConstPtr& model,
        const rclcpp::Node::SharedPtr& node,
        const std::string& parameter_namespace) override
    {
        robot_model_ = model;
        node_ = node;
        parameter_namespace_ = parameter_namespace;
        param_listener_ = std::make_shared<ktopt_interface::ParamListener>(node, parameter_namespace);
        return true;
    }

    bool canServiceRequest(
        const planning_interface::MotionPlanRequest& req
    ) const override
    {
        if (req.goal_constraints.empty())
        {
            RCLCPP_ERROR(getLogger(), "Invalid goal constraints");
            return false;
        }

        if (req.group_name.empty() || !robot_model_-> hasJointModelGroup(req.group_name))
        {
            RCLCPP_ERROR(getLogger(), "Invalid joint group '%s'", req.group_name.c_str());
            return false;
        }
        return false;
    }

    std::string getDescription() const override
    {
        return "Kinematic Trajectory Optimization Planner";
    }

    void getPlanningAlgorithms(
        std::vector<std::string>& algs) const override
    {
        algs.clear();
        algs.push_back("ktopt");
    }

    planning_interface::PlanningContextPtr getPlanningContext(
        const planning_scene::PlanningSceneConstPtr& planning_scene,
        const planning_interface::MotionPlanRequest& req,
        moveit_msgs::msg::MoveItErrorCodes& error_code
    ) const override
    {
        if (!canServiceRequest(req))
        {
            error_code.val = moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED;
            return nullptr;
        }

        const auto params = param_listener_->get_params();
        std::shared_ptr<KTOptPlanningContext> planning_context = std::make_shared<KTOptPlanningContext>(
            "KTOPT",
            req.group_name,
            params);
        planning_context->setPlanningScene(planning_scene);
        planning_context->setMotionPlanRequest(req);

        if (!params.path_marker_topic.empty())
        {
            auto path_publisher = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                params.path_marker_topic,
                rclcpp::SystemDefaultsQoS());
            planning_context->setPathPublisher(path_publisher);
        }

        return planning_context;
    }

private:
    moveit::core::RobotModelConstPtr robot_model_;
    rclcpp::Node::SharedPtr node_;
    std::string parameter_namespace_;
    std::shared_ptr<ktopt_interface::ParamListener> param_listener_;

};

} // namespace ktopt_interface

// register the KTOptPlannerManager class as a plugin
CLASS_LOADER_REGISTER_CLASS(
    ktopt_interface::KTOptPlannerManager,
    planning_interface::PlannerManager);

