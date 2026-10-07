#ifndef TREE_HHCM_MISSIONTRACKER_H
#define TREE_HHCM_MISSIONTRACKER_H

#include <tree_hhcm/common/common.h>
#include <tree_hhcm/common/mission_group.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// ros
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace tree {

/**
 * 
 *  @brief Publishes the current state of the mission to a ROS topic
 *
 */

class MissionTracker : public BT::StatefulActionNode
{

public:

    MissionTracker(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

private:

    void publish();

    std::string to_json(const MissionGroupRegistry::State& state) const;

    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;
    MissionGroupRegistry::State _published;
    bool _ever_published = false;
    Printer _p;

};

}

#endif // TREE_HHCM_MISSIONTRACKER_H
