#ifndef tree_hhcm_ROBOT_PLAY_TRAJ_H
#define tree_hhcm_ROBOT_PLAY_TRAJ_H

#include <tree_hhcm/common/common.h>
#include <xbot2_interface/xbotinterface2.h>
#include <string>
#include <vector>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class RobotPlayTrajectory : public BT::StatefulActionNode
{

public:

    RobotPlayTrajectory(std::string name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override {}

    static BT::PortsList providedPorts();

private:

    Eigen::VectorXd sampleAt(double time) const;

    XBot::ModelInterface::Ptr _model;
    std::vector<int> _jointIdx;
    std::vector<Eigen::VectorXd> _points;
    double _dt;
    double _rate;
    double _time;
    double _duration;
    Printer _p;
};

}

#endif // tree_hhcm_ROBOT_PLAY_TRAJ_H
