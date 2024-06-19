#ifndef KTOPT_PLANNING_CONTEXT_H
#define KTOPT_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include "ktopt_interface/ktopt_interface.h"

namespace ktopt_interface
{
class KTOptPlanningContext : public planning_interface::PlanningContext
{
public:
    KTOptPlanningContext(
        const std::string& name,
        const std::string& group,
        const moveit::core::RobotModelConstPtr& model
    );
    ~KTOptPlanningContext() override
    {
    };

    bool solve(planning_interface:MotionPlanResponse& res) override;
    bool solve(planning_interface:MotionPlanDetailResponse& res) override;

    bool terminate() override;
    void clear() override;

private:
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;
    KTOptInterfacePtr ktopt_interface_;
};
} // namespace ktopt_interface

#endif //KTOPT_PLANNING_CONTEXT_H

