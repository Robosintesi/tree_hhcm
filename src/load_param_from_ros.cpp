#include <tree_hhcm/ros/load_param_from_ros.h>

#include <chrono>

using namespace tree;

namespace {

template <typename Future>
bool is_ready(const Future& future)
{
    return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

}

LoadParamFromRos::LoadParamFromRos(const std::string &name, const BT::NodeConfiguration &config):
    BT::StatefulActionNode(name, config), _p(*this)
{
}

BT::NodeStatus LoadParamFromRos::onStart()
{
    if(!getInput("node", _node))
    {
        throw BT::RuntimeError("LoadParamFromRos: missing required input [node]");
    }

    if(!getInput("remote_node", _remote_node))
    {
        throw BT::RuntimeError("LoadParamFromRos: missing required input [remote_node]");
    }

    if(!getInput("prefix", _prefix))
    {
        throw BT::RuntimeError("LoadParamFromRos: missing required input [prefix]");
    }

    _output_name = getInput<std::string>("output_name").value_or(_prefix);

    _timeout = 5.0;
    getInput("timeout", _timeout);

    _client = std::make_shared<rclcpp::AsyncParametersClient>(_node, _remote_node);
    _list_future = {};
    _get_future = {};
    _time = 0.0;

    _p.cout() << "loading parameters " << _prefix << ".* of " << _remote_node << "\n";

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus LoadParamFromRos::onRunning()
{
    rclcpp::spin_some(_node);

    if(_time > _timeout)
    {
        _p.cerr() << "timed out: no parameters from " << _remote_node << ", is it running?\n";

        return BT::NodeStatus::FAILURE;
    }

    _time += Globals::instance().tree_dt;

    // the parameter services of the remote node may still be coming up
    if(!_list_future.valid())
    {
        if(_client->service_is_ready())
        {
            _list_future = _client->list_parameters(
                {_prefix}, rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
        }

        return BT::NodeStatus::RUNNING;
    }

    if(!is_ready(_list_future))
    {
        return BT::NodeStatus::RUNNING;
    }

    if(!_get_future.valid())
    {
        const auto& names = _list_future.get().names;

        if(names.empty())
        {
            _p.cerr() << "no parameter under '" << _prefix << "' on " << _remote_node << "\n";

            return BT::NodeStatus::FAILURE;
        }

        _get_future = _client->get_parameters(names);

        return BT::NodeStatus::RUNNING;
    }

    if(!is_ready(_get_future))
    {
        return BT::NodeStatus::RUNNING;
    }

    for(const auto& param : _get_future.get())
    {
        const auto key = _output_name + param.get_name().substr(_prefix.size());

        if(!set_param_to_blackboard(key, param))
        {
            _p.cerr() << "parameter " << param.get_name() << " has unsupported type "
                      << param.get_type_name() << "\n";

            return BT::NodeStatus::FAILURE;
        }

        _p.cout() << key << " := " << param.value_to_string() << "\n";
    }

    return BT::NodeStatus::SUCCESS;
}

void LoadParamFromRos::onHalted()
{
    _list_future = {};
    _get_future = {};
    _client.reset();
}

bool LoadParamFromRos::set_param_to_blackboard(const std::string& key, const rclcpp::Parameter& param)
{
    auto& blackboard = *config().blackboard;

    switch(param.get_type())
    {
        case rclcpp::ParameterType::PARAMETER_BOOL:
            blackboard.set(key, param.as_bool());
            return true;

        case rclcpp::ParameterType::PARAMETER_INTEGER:
            blackboard.set(key, static_cast<int>(param.as_int()));
            return true;

        case rclcpp::ParameterType::PARAMETER_DOUBLE:
            blackboard.set(key, param.as_double());
            return true;

        case rclcpp::ParameterType::PARAMETER_STRING:
            blackboard.set(key, param.as_string());
            return true;

        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
            blackboard.set(key, param.as_bool_array());
            return true;

        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
        {
            const auto& values = param.as_integer_array();
            blackboard.set(key, std::vector<int>(values.begin(), values.end()));
            return true;
        }

        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
            blackboard.set(key, param.as_double_array());
            return true;

        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
            blackboard.set(key, param.as_string_array());
            return true;

        default:
            return false;
    }
}

BT::PortsList LoadParamFromRos::providedPorts()
{
    return {
        BT::InputPort<rclcpp::Node::SharedPtr>("node",
            "ros2 node, as provided by NodeProvider"),

        BT::InputPort<std::string>("remote_node",
            "node that holds the parameters, e.g. /xbot2_gui_server"),

        BT::InputPort<std::string>("prefix",
            "loads the parameters <prefix>.* of the remote node"),

        BT::InputPort<std::string>("output_name",
            "blackboard prefix of the loaded parameters; defaults to prefix"),

        BT::InputPort<double>("timeout", 5.0,
            "seconds to wait for the parameters of the remote node"),
    };
}
