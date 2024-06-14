#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit_msgs/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

#include "ktopt_interface/ktopt_planning_context.h"
#include "ktopt_interface/ktopt_interface.h"

namespace ktopt_interface
{
KTOptPlanningContext::KTOptPlanningContext(
    const std::string& context_name,
    const std::string& group_name,
    const moveit::core::RobotModelConstPtr& model
) : planning_interface::PlanningContext(context_name, group_name), robot_model_(model)
{
    ktopt_interface_ = KTOptInterfacePtr(new KTOptInterface());
}

bool KTOptPlanningContext::solve(
    planning_interace::MotionPlanDetailedResponse& res)
{
    moveit_msgs::MotionPlanDetailedResponse res_msg;
    return true;
}

bool KTOptPlanningContext::terminate()
{
    return true;
}

void KTOptPlanningContext::clear()
{
    // do something
}
} // namespace ktopt_interface
