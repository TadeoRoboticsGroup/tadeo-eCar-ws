#include <behaviortree_cpp_v3/behavior_tree.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "tadeo_ecar_behavior/behavior_types.hpp"

namespace tadeo_ecar_behavior
{

// Placeholder para acciones de exploración
class ExploreAreaAction : public BT::SyncActionNode
{
public:
    ExploreAreaAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("explore_area_bt_node");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("result", "Exploration result")
        };
    }

    BT::NodeStatus tick() override
    {
        // Implementación placeholder
        setOutput("result", "Area explored (placeholder)");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

// Placeholder para acción de docking
class DockToStationAction : public BT::SyncActionNode
{
public:
    DockToStationAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::SyncActionNode(name, config)
    {
        node_ = rclcpp::Node::make_shared("dock_to_station_bt_node");
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::string>("result", "Docking result")
        };
    }

    BT::NodeStatus tick() override
    {
        // Implementación placeholder
        setOutput("result", "Docking completed (placeholder)");
        return BT::NodeStatus::SUCCESS;
    }

private:
    rclcpp::Node::SharedPtr node_;
};

} // namespace tadeo_ecar_behavior