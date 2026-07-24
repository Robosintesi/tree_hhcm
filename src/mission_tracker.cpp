#include <tree_hhcm/ros/mission_tracker.h>

#include <sstream>

using namespace tree;

namespace {

std::string json_escape(const std::string& s)
{
    std::string out;
    out.reserve(s.size());

    for(char c : s)
    {
        if(c == '"' || c == '\\')
        {
            out += '\\';
        }
        out += c;
    }

    return out;
}

}

MissionTracker::MissionTracker(const std::string& name, const BT::NodeConfiguration& config):
    BT::StatefulActionNode(name, config),
    _p(*this)
{
}

BT::PortsList MissionTracker::providedPorts()
{
    return {
        BT::InputPort<rclcpp::Node::SharedPtr>("node", "the ROS2 node"),
        BT::InputPort<std::string>("topic", "mission_progress", "topic to publish the mission state on"),
    };
}

BT::NodeStatus MissionTracker::onStart()
{
    if(!getInput("node", _node))
    {
        throw BT::RuntimeError("missing required input [node]");
    }

    std::string topic = "mission_progress";
    getInput("topic", topic);

    _pub = _node->create_publisher<std_msgs::msg::String>(
        topic,
        rclcpp::QoS(1).transient_local());

    _p.cout() << "tracking " << MissionGroupRegistry::instance().size()
              << " mission group(s), publishing on [" << _pub->get_topic_name() << "]\n";

    publish();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MissionTracker::onRunning()
{
    publish();

    return BT::NodeStatus::RUNNING;
}

void MissionTracker::onHalted()
{
    // publish a final state
    if(!_pub)
    {
        return;
    }

    _published = MissionGroupRegistry::State{};
    _ever_published = true;

    std_msgs::msg::String msg;
    msg.data = to_json(_published);
    _pub->publish(msg);
}

void MissionTracker::publish()
{
    auto state = MissionGroupRegistry::instance().state();

    if(_ever_published && state == _published)
    {
        return;
    }

    _published = std::move(state);
    _ever_published = true;

    std_msgs::msg::String msg;
    msg.data = to_json(_published);
    _pub->publish(msg);
}

std::string MissionTracker::to_json(const MissionGroupRegistry::State& state) const
{
    std::ostringstream ss;

    ss << "{"
       << "\"stage\":\"" << json_escape(state.stage) << "\","
       << "\"previous\":\"" << json_escape(state.previous) << "\","
       << "\"current\":\"" << json_escape(state.current) << "\","
       << "\"next\":\"" << json_escape(state.next) << "\","
       << "\"cycle_label\":\"" << json_escape(state.cycle_label) << "\","
       << "\"cycle\":" << state.cycle << ","
       << "\"cycle_count\":" << state.cycle_count
       << "}";

    return ss.str();
}
