#include <moveit/robot_state/conversions.h>
#include <moveit/planning_interface/planning_interface.h>
//#include <moveit_msgs/msg/MotionPlanRequest.h>
#include <moveit/planning_scene/planning_scene.h>

#include "ktopt_interface/ktopt_planning_context.h"
#include "ktopt_interface/ktopt_interface.h"

namespace ktopt_interface
{

void KTOptPlanningContext::solve(
    planning_interface::MotionPlanDetailedResponse& res)
{
    return;
}

void KTOptPlanningContext::solve(
    planning_interface::MotionPlanResponse& res)
{
    return;
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
