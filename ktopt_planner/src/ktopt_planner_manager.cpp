#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_interface/planning_response.h>
#include <moveit/planning_scene/planning_scene.h>
#include <class_loader/class_loader.hpp>

namespace ktopt_interface
{
class KTOptPlannerManager : public planning_interface::PlannerManager
{
public:
    KTOptPlannerManager() : planning_interface::PlannerManager()
    {
    }

    bool initialize(
        const moveit::core::RobotModelConstPtr& model,
        const std::string& /*ns*/) override
    {
        for (const std::string& gpName : model->getJointModelGroupNames())
        {
        }
        return true;
    }

    bool canServiceRequest(
        const moveit_msgs::MotionPlanRequest7 req
    ) const override
    {
        return req.trajectory_contraints.constraints.empty();
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
        moveit_msgs::MoveItErrorCodes& error_code
    ) const override
    {
        error_code.val = moveit_msgs::MoveitErrorCodes::SUCCESS;
    }

protected:
    std::map<std::string, KTOptPlanningContextPtr> planning_contexts_;
};

} // namespace ktopt_interface

// register the KTOptPlannerManager class as a plugin
CLASS_LOADER_TEGISTER_CLASS(
    ktopt_interface::KTOptPlannerManager,
    planning_interface::PlannerManager);

