#include <tree_hhcm/robot/robot_play_trajectory.h>

#include <map>
#include <cmath>

using namespace tree;
using namespace std::string_literals;

namespace {

// Trajectory files are read once and kept: a mission replays the same handful of
// motions many times, and they hold a few hundred waypoints each.
std::map<std::string, YAML::Node> trajectoryCache;

const YAML::Node& loadTrajectory(const std::string &path)
{
    auto it = trajectoryCache.find(path);
    if(it != trajectoryCache.end())
    {
        return it->second;
    }

    std::string resolved = Globals::instance().parse_shell(path);
    auto res = trajectoryCache.emplace(path, YAML::LoadFile(resolved));
    return res.first->second;
}

}

RobotPlayTrajectory::RobotPlayTrajectory(std::string name, const BT::NodeConfiguration &config)
:
    BT::StatefulActionNode(name, config),
    _dt(0.0),
    _rate(1.0),
    _time(0.0),
    _duration(0.0),
    _p(*this)
{
}

BT::NodeStatus RobotPlayTrajectory::onStart()
{
    if(!getInput("model", _model))
    {
        throw BT::RuntimeError("RobotPlayTrajectory: missing required input [model]");
    }

    std::string file;
    if(!getInput("file", file))
    {
        throw BT::RuntimeError("RobotPlayTrajectory: missing required input [file]");
    }

    std::string phase;
    if(!getInput("phase", phase))
    {
        throw BT::RuntimeError("RobotPlayTrajectory: missing required input [phase]");
    }

    const YAML::Node &traj = loadTrajectory(file);

    auto jointNames = traj["joint_names"].as<std::vector<std::string>>();
    _jointIdx.clear();
    for(const auto &n : jointNames)
    {
        _jointIdx.push_back(_model->getJointInfo(n).iq);
    }

    _dt = traj["dt"].as<double>();
    _rate = getInput<double>("rate").value_or(1.0);
    if(_rate <= 0.0)
    {
        throw BT::RuntimeError("RobotPlayTrajectory: [rate] must be positive");
    }

    YAML::Node points;
    for(const auto &s : traj["phases"])
    {
        if(s["name"].as<std::string>() == phase)
        {
            points = s["points"];
            break;
        }
    }
    if(!points)
    {
        throw BT::RuntimeError("RobotPlayTrajectory: no phase "s + phase + " in " + file);
    }

    _points.clear();
    _points.reserve(points.size());
    for(const auto &row : points)
    {
        auto v = row.as<std::vector<double>>();
        if(v.size() != jointNames.size())
        {
            throw BT::RuntimeError("RobotPlayTrajectory: waypoint width does not match joint_names");
        }
        _points.push_back(Eigen::Map<Eigen::VectorXd>(v.data(), v.size()));
    }
    if(_points.size() < 2)
    {
        throw BT::RuntimeError("RobotPlayTrajectory: phase "s + phase + " has fewer than 2 waypoints");
    }

    _duration = (_points.size() - 1) * _dt;

    double maxStartError = getInput<double>("max_start_error").value_or(0.05);
    Eigen::VectorXd qcurrent;
    _model->getJointPosition(qcurrent);
    double startError = 0.0;
    for(size_t i = 0; i < _jointIdx.size(); ++i)
    {
        startError = std::max(startError, std::fabs(qcurrent(_jointIdx[i]) - _points.front()(i)));
    }
    if(startError > maxStartError)
    {
        _p.cout() << "RobotPlayTrajectory: start configuration is " << startError
                  << " rad away from the first waypoint of " << phase
                  << " (limit " << maxStartError << ")\n";
        return BT::NodeStatus::FAILURE;
    }

    _p.cout() << "RobotPlayTrajectory: " << phase << " from " << file
              << ", " << _points.size() << " waypoints, "
              << _duration / _rate << " s, start error " << startError << " rad\n";

    _time = 0.0;
    return BT::NodeStatus::RUNNING;
}

Eigen::VectorXd RobotPlayTrajectory::sampleAt(double time) const
{
    double s = std::min(std::max(time, 0.0), _duration) / _dt;
    size_t i = static_cast<size_t>(std::floor(s));
    if(i >= _points.size() - 1)
    {
        return _points.back();
    }
    double a = s - static_cast<double>(i);
    return (1.0 - a) * _points[i] + a * _points[i + 1];
}

BT::NodeStatus RobotPlayTrajectory::onRunning()
{
    Eigen::VectorXd wp = sampleAt(_time);

    Eigen::VectorXd q;
    _model->getJointPosition(q);
    for(size_t i = 0; i < _jointIdx.size(); ++i)
    {
        q(_jointIdx[i]) = wp(i);
    }

    _model->setJointPosition(q);
    _model->update();

    setOutput("q", q);

    _time += Globals::instance().tree_dt * _rate;

    if(_time > _duration)
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

BT::PortsList RobotPlayTrajectory::providedPorts()
{
    return {
        BT::InputPort<XBot::ModelInterface::Ptr>("model"),
        BT::InputPort<std::string>("file", "Trajectory file planned by anime_curobo/motion_planning.py"),
        BT::InputPort<std::string>("phase", "Phase to play: approach, grasp or lift"),
        BT::InputPort<double>("rate", "Playback speed scale, 1.0 plays at the planned speed"),
        BT::InputPort<double>("max_start_error", "Fail if any joint is farther than this [rad] from the first waypoint. Default 0.05"),
        BT::OutputPort<Eigen::VectorXd>("q", "Current joint configuration")
    };
}
