#include <tree_hhcm/robot/robot_set_joint_impedance.h>

#include <algorithm>
#include <vector>

namespace {

constexpr auto command_topic = "/xbotcore/command";
constexpr auto joint_state_topic = "/xbotcore/joint_states";

std::vector<std::string> commanded_joints(const std::map<std::string, double>& stiffness,
                                          const std::map<std::string, double>& damping)
{
    std::vector<std::string> names;
    names.reserve(stiffness.size() + damping.size());

    for(const auto& [jname, gain] : stiffness)
    {
        names.push_back(jname);
    }

    for(const auto& [jname, gain] : damping)
    {
        names.push_back(jname);
    }

    std::sort(names.begin(), names.end());
    names.erase(std::unique(names.begin(), names.end()), names.end());

    return names;
}

double blend(double tau)
{
    tau = std::clamp(tau, 0.0, 1.0);

    return ((6.0*tau - 15.0)*tau + 10.0)*tau*tau*tau;
}

std::optional<std::size_t> index_of(const std::vector<std::string>& names,
                                    const std::string& name)
{
    const auto it = std::find(names.begin(), names.end(), name);

    if(it == names.end())
    {
        return std::nullopt;
    }

    return static_cast<std::size_t>(std::distance(names.begin(), it));
}

} // anonymous namespace

namespace tree {

RobotSetJointImpedance::RobotSetJointImpedance(const std::string& name,
                                               const BT::NodeConfiguration& config):
    BT::StatefulActionNode(name, config), _p(*this)
{

}

RobotSetJointImpedance::JointCommand
RobotSetJointImpedance::make_command(const GainMap& stiffness,
                                     const GainMap& damping)
{
    const auto names = commanded_joints(stiffness, damping);

    JointCommand cmd;
    cmd.name = names;
    cmd.ctrl_mode.assign(names.size(), 0);
    cmd.stiffness.assign(names.size(), 0.0f);
    cmd.damping.assign(names.size(), 0.0);

    for(std::size_t i = 0; i < names.size(); ++i)
    {
        if(const auto it = stiffness.find(names[i]); it != stiffness.end())
        {
            cmd.stiffness[i] = static_cast<float>(it->second);
            cmd.ctrl_mode[i] |= stiffness_bit;
        }

        if(const auto it = damping.find(names[i]); it != damping.end())
        {
            cmd.damping[i] = it->second;
            cmd.ctrl_mode[i] |= damping_bit;
        }
    }

    return cmd;
}

bool RobotSetJointImpedance::capture_initial_gains()
{
    const auto& js = *_joint_state;

    _stiffness_0.assign(_target.name.size(), 0.0f);
    _damping_0.assign(_target.name.size(), 0.0);

    for(std::size_t i = 0; i < _target.name.size(); ++i)
    {
        const auto j = index_of(js.name, _target.name[i]);

        if(!j || *j >= js.stiffness.size() || *j >= js.damping.size())
        {
            _p.cerr() << "joint '" << _target.name[i]
                      << "' is not reported on " << joint_state_topic << "\n";

            return false;
        }

        _stiffness_0[i] = js.stiffness[*j];
        _damping_0[i] = js.damping[*j];

        if(!(_target.ctrl_mode[i] & stiffness_bit))
        {
            _target.stiffness[i] = _stiffness_0[i];
        }

        if(!(_target.ctrl_mode[i] & damping_bit))
        {
            _target.damping[i] = _damping_0[i];
        }
    }

    return true;
}

BT::NodeStatus RobotSetJointImpedance::onStart()
{
    if(!getInput("node", _node))
    {
        throw BT::RuntimeError("RobotSetJointImpedance: missing required input [node]");
    }

    GainMap stiffness, damping;
    getInput("stiffness", stiffness);
    getInput("damping", damping);

    if(stiffness.empty() && damping.empty())
    {
        throw BT::RuntimeError("RobotSetJointImpedance: stiffness/damping map is empty");
    }

    for(const auto* gains : {&stiffness, &damping})
    {
        for(const auto& [jname, gain] : *gains)
        {
            if(!(gain >= 0.0))
            {
                throw BT::RuntimeError("RobotSetJointImpedance: invalid gain for joint '" + jname + "'");
            }
        }
    }

    _target = make_command(stiffness, damping);
    _cmd = _target;

    _transition_time = 2.0;
    getInput("transition_time", _transition_time);

    if(!(_transition_time > 1.0))
    {
        throw BT::RuntimeError("RobotSetJointImpedance: [transition_time] must be greater than 1.0");
    }

    _timeout = 2.0;
    getInput("timeout", _timeout);

    _time = 0.0;
    _transition_start.reset();
    _joint_state.reset();

    _pub = _node->create_publisher<JointCommand>(command_topic, rclcpp::SensorDataQoS());

    _sub = _node->create_subscription<JointState>(
        joint_state_topic, rclcpp::SensorDataQoS(),
        [this](JointState::ConstSharedPtr msg)
        {
            _joint_state = std::move(msg);
        });

    _p.cout() << "ramping impedance of " << _target.name.size()
              << " joint(s) over " << _transition_time << "s\n";

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RobotSetJointImpedance::onRunning()
{
    rclcpp::spin_some(_node);

    if(!_joint_state)
    {
        if(_time > _timeout)
        {
            _p.cerr() << "timed out: no joint state on " << joint_state_topic
                      << ", is xbot2 running?\n";

            return BT::NodeStatus::FAILURE;
        }

        _time += Globals::instance().tree_dt;

        return BT::NodeStatus::RUNNING;
    }

    if(!_transition_start)
    {
        if(!capture_initial_gains())
        {
            return BT::NodeStatus::FAILURE;
        }

        _transition_start = _time;
    }

    const double alpha = _transition_time > 0.0
        ? blend((_time - *_transition_start)/_transition_time)
        : 1.0;

    for(std::size_t i = 0; i < _target.name.size(); ++i)
    {
        _cmd.stiffness[i] = static_cast<float>(
            _stiffness_0[i]*(1.0 - alpha) + _target.stiffness[i]*alpha);

        _cmd.damping[i] = _damping_0[i]*(1.0 - alpha) + _target.damping[i]*alpha;
    }

    _cmd.header.stamp = _node->now();
    _pub->publish(_cmd);

    _time += Globals::instance().tree_dt;

    return alpha < 1.0 ? BT::NodeStatus::RUNNING : BT::NodeStatus::SUCCESS;
}

void RobotSetJointImpedance::onHalted()
{
    _sub.reset();
    _pub.reset();
    _joint_state.reset();
    _transition_start.reset();
}

BT::PortsList RobotSetJointImpedance::providedPorts()
{
    return {
        BT::InputPort<rclcpp::Node::SharedPtr>("node",
            "ros2 node, as provided by NodeProvider"),

        BT::InputPort<GainMap>("stiffness",
            "joint name to stiffness map, e.g. 'J1_A: 2000, J2_A: 1000'"),

        BT::InputPort<GainMap>("damping",
            "joint name to damping map, e.g. 'J1_A: 20, J2_A: 10'"),

        BT::InputPort<double>("transition_time", 2.0,
            "seconds spent ramping the gains from their measured value to the "
            "reference; 0 commands a step, which will jerk the robot"),

        BT::InputPort<double>("timeout", 2.0,
            "seconds to wait for the first joint state before giving up"),
    };
}

} // namespace tree
