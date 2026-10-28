#ifndef _tree_hhcm_LOADPARAMFROMROS_H
#define _tree_hhcm_LOADPARAMFROMROS_H

#include <tree_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// ros
#include <rclcpp/rclcpp.hpp>

#include <future>
#include <string>
#include <vector>

namespace tree {

/**
 * @brief Copies the parameters under a prefix of another ROS node to the blackboard.
 * 
 *   <LoadParamFromRos node="{ros_node}"
 *                     remote_node="/xbot2_guii_server"
 *                     prefix="mission"
 *                     output_name="mission" />
 *
 * Fails if the remote node does not answer within the timeout, or has no parameter under the prefix.
 *
 */

class LoadParamFromRos : public BT::StatefulActionNode
{

public:

    LoadParamFromRos(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    static BT::PortsList providedPorts();

private:

    // false if the parameter type has no blackboard counterpart
    bool set_param_to_blackboard(const std::string& key, const rclcpp::Parameter& param);

    rclcpp::Node::SharedPtr _node;
    rclcpp::AsyncParametersClient::SharedPtr _client;

    std::shared_future<rcl_interfaces::msg::ListParametersResult> _list_future;
    std::shared_future<std::vector<rclcpp::Parameter>> _get_future;

    std::string _remote_node;
    std::string _prefix;
    std::string _output_name;

    // tick-driven time, like the other ros nodes
    double _time = 0.0;
    double _timeout = 5.0;

    Printer _p;

};

}

#endif // _tree_hhcm_LOADPARAMFROMROS_H
