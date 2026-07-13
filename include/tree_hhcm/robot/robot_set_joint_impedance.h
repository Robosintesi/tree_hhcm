#ifndef _tree_hhcm_ROBOTSETJOINTIMPEDANCE_H
#define _tree_hhcm_ROBOTSETJOINTIMPEDANCE_H

#include <tree_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

// ros
#include <rclcpp/rclcpp.hpp>
#include <xbot_msgs/msg/joint_command.hpp>
#include <xbot_msgs/msg/joint_state.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace tree {

/**
 * @brief Sets the joint impedance (stiffness and/or damping) of a subset of the robot joints.
 *
 *   <RobotSetJointImpedance node="{ros_node}"
 *                           stiffness="J1_A: 2000, J2_A: 1000"
 *                           damping="J1_A: 20"
 *                           transition_time="2.0" />
 *
 */
class RobotSetJointImpedance : public BT::StatefulActionNode
{

public:

    RobotSetJointImpedance(const std::string& name, const BT::NodeConfiguration& config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override;

    static BT::PortsList providedPorts();

private:

    using JointCommand = xbot_msgs::msg::JointCommand;
    using JointState = xbot_msgs::msg::JointState;
    using GainMap = std::map<std::string, double>;

    static constexpr std::uint8_t stiffness_bit = 8;
    static constexpr std::uint8_t damping_bit = 16;

    static JointCommand make_command(const GainMap& stiffness,
                                     const GainMap& damping);

    bool capture_initial_gains();

    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<JointCommand>::SharedPtr _pub;
    rclcpp::Subscription<JointState>::SharedPtr _sub;

    JointState::ConstSharedPtr _joint_state;

    JointCommand _target;
    JointCommand _cmd;

    std::vector<float> _stiffness_0;
    std::vector<double> _damping_0;

    // tick-driven time (tree_dt per tick): pausing the executor
    // freezes the ramp instead of making it jump forward on resume
    double _time = 0.0;
    std::optional<double> _transition_start;
    double _timeout = 2.0;

    double _transition_time = 2.0;

    Printer _p;

};

}

#endif // _tree_hhcm_ROBOTSETJOINTIMPEDANCE_H
