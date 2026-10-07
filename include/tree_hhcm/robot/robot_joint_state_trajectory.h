#ifndef tree_hhcm_ROBOT_JSTRAJ_H
#define tree_hhcm_ROBOT_JSTRAJ_H

#include <tree_hhcm/common/common.h>
#include <xbot2_interface/xbotinterface2.h>
#include <set>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class RobotJointStateTrajectory : public BT::StatefulActionNode
{

public:

    RobotJointStateTrajectory(std::string name, const BT::NodeConfiguration &config);

    BT::NodeStatus onStart() override;

    BT::NodeStatus onRunning() override;

    void onHalted() override {}

    static BT::PortsList providedPorts();

private:

    XBot::ModelInterface::Ptr _model;
    Eigen::VectorXd _qstart, _qgoal, _deltaq;
    double _duration;
    double _time;
    std::set<int> _keep;   // indices of the controlled joints
    bool _compose;         // write only controlled joints 
    Printer _p;
};

}

#endif // tree_hhcm_ROBOT_MOVE_H