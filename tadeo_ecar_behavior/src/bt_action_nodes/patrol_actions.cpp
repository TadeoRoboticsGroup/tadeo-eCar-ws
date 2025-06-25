#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "tadeo_ecar_behavior/behavior_types.hpp"

namespace tadeo_ecar_behavior
{

// Placeholder para acciones de patrullaje
class ExecutePatrolAction : public BT::SyncActionNode
{
public:
    ExecutePatrolAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("execute_patrol_bt_node");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("result", "Patrol execution result")
        };
    }

    BT::NodeStatus tick() override
    {
        // Implementación placeholder
        setOutput("result", "Patrol executed (placeholder)");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

} // namespace tadeo_ecar_behavior