#ifndef TREE_HHCM_COMMON_STRING_FORMAT_H
#define TREE_HHCM_COMMON_STRING_FORMAT_H

#include <tree_hhcm/common/common.h>

// behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/action_node.h>

namespace tree {

class StringFormat : public BT::SyncActionNode
{

public:

    StringFormat(std::string name, const BT::NodeConfiguration &config);

    BT::NodeStatus tick() override;

    static BT::PortsList providedPorts();

private:

    Printer _p;
};

}

#endif // TREE_HHCM_COMMON_STRING_FORMAT_H
