#ifndef KTOPT_PLANNING_CONTEXT_H
#define KTOPT_PLANNING_CONTEXT_H

#include <moveit/planning_interface/planning_interface.h>
#include "ktopt_interface/ktopt_interface.h"
#include <drake_moveit_parameters.hpp>

namespace ktopt_interface
{
class KTOptPlanningContext : public planning_interface::PlanningContext
{
public:
    KTOptPlanningContext(
        const std::string& name,
        const std::string& group_name,
        const ktopt_interface::Params& params);
    ~KTOptPlanningContext() override
    {
    };

    void solve(planning_interface::MotionPlanResponse& res) override;
    void solve(planning_interface::MotionPlanDetailedResponse& res) override;

    bool terminate() override;
    void clear() override;

    void setPathPublisher(
        const std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>& path_publisher
    );
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> getPathPublisher();

private:
    const ktopt_interface::Params params_;
    std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>> path_publisher_;
};
} // namespace ktopt_interface

#endif //KTOPT_PLANNING_CONTEXT_H

