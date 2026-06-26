#include <tree_hhcm/robot/robot_joint_state_trajectory.h>
#include <set>

using namespace tree;

tree::RobotJointStateTrajectory::RobotJointStateTrajectory(std::string name, const BT::NodeConfiguration &config)
:
    BT::StatefulActionNode(name, config),
    _p(*this)
{
}

BT::NodeStatus tree::RobotJointStateTrajectory::onStart()
{

    if(!getInput("model", _model))
    {
        throw BT::RuntimeError("missing required input [model]");
    }

    _keep.clear();
    _compose = false;
    getInput("compose", _compose);

    _model->getJointPosition(_qstart);

    

    if(getInput("q_start", _qstart))
    {
        _p.cout() << "overriding q_start with provided input q_start" << _qstart.transpose().format(2) << std::endl;
    }

    std::vector<std::string> joint_names;
    getInput("joint_names", joint_names);

    std::string q_goal_name;
    if(getInput("q_goal_name", q_goal_name))
    {
        _p.cout() << "overriding q_goal with provided q_goal_name " << std::quoted(q_goal_name) << std::endl;

        _qgoal = _model->getRobotState(q_goal_name);
    }
    else
    {
        Eigen::VectorXd q_goal_in;
        if(!getInput("q_goal", q_goal_in))
        {
            throw BT::RuntimeError("missing required input [q_goal] or [q_goal_name]");
        }

        if(!joint_names.empty())
        {
            if(q_goal_in.size() != (int)joint_names.size())
            {
                throw BT::RuntimeError("q_goal size (" + std::to_string(q_goal_in.size()) + ") must match joint_names size (" + std::to_string(joint_names.size()) + ")");
            }
            // Override q_goal with current robot state
            _model->getJointPosition(_qgoal);
            // Set only the joints specified in joint_names
            for(size_t i = 0; i < joint_names.size(); ++i)
            {
                int idx = _model->getJointInfo(joint_names[i]).iq;
                _qgoal(idx) = q_goal_in(i);
            }
            // Set delta q for unspecified joints to zero
            for(const auto &name : joint_names)
            {
                int idx = _model->getJointInfo(name).iq;
                _p.cout() << "Moving joint " << name << " (idx " << idx << ")\n";
                _keep.insert(idx);
            }
            for(int i = 0; i < _deltaq.size(); ++i)
            {
                if(!_keep.count(i))
                    _deltaq(i) = 0.0;
            }
        }
        else
        {
            _qgoal = q_goal_in;
        }
    }

    if(_compose && _keep.empty())
    {
        throw BT::RuntimeError("compose=true requires [joint_names]");
    }

    if(!getInput("duration", _duration))
    {
        throw BT::RuntimeError("missing required input [duration]");
    }

    _time = 0.0;

    _deltaq = _model->difference(_qgoal, _qstart);

    std::vector<int> fixed_idx;
    if(getInput("fixed_idx", fixed_idx))
    {
        for(auto idx : fixed_idx)
        {
            _p.cout() << "Fixing joint " << _model->getVNames()[idx] << " (idx " << idx << ")\n";
            _deltaq(idx) = 0.0;
        }
    }

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus tree::RobotJointStateTrajectory::onRunning()
{
    // normalized time and quintic interpolation
    double tau = _time / _duration;
    double alpha = ((6*tau - 15)*tau + 10)*tau*tau*tau;

    // integrate interpolated delta q from q start
    Eigen::VectorXd q = _model->sum(_qstart, alpha*_deltaq);

    if(_compose && !_keep.empty())
    {
        // move only the controlled joints
        Eigen::VectorXd qcurrent;
        _model->getJointPosition(qcurrent);
        for(int idx : _keep)
            qcurrent(idx) = q(idx);
        q = qcurrent;
    }

    // set to model and update
    _model->setJointPosition(q);
    _model->update();

    setOutput("q", q);

    _time += Globals::instance().tree_dt;

    if(_time > _duration)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

BT::PortsList tree::RobotJointStateTrajectory::providedPorts()
{
    return {
        BT::InputPort<XBot::ModelInterface::Ptr>("model"),
        BT::InputPort<Eigen::VectorXd>("q_start", "Starting joint configuration. If not provided, the current robot configuration is used"),
        BT::InputPort<Eigen::VectorXd>("q_goal", "Goal joint configuration"),
        BT::InputPort<std::string>("q_goal_name", "Name of the robot state to use as goal joint configuration. Overrides the current robot configuration"),
        BT::InputPort<double>("duration", "Duration of the trajectory [s]"),
        BT::InputPort<std::vector<int>>("fixed_idx", "Indices of joints to keep fixed during the trajectory"),
        BT::InputPort<std::vector<std::string>>("joint_names", "Names of the joints to control"),
        BT::InputPort<bool>("compose", "Move only [joint_names] over the live state, leaving other joints at the current state. Requires [joint_names]. Default false."),
        BT::OutputPort<Eigen::VectorXd>("q", "Current joint configuration")
    };
}
