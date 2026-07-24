#ifndef TREE_HHCM_MISSIONGROUP_H
#define TREE_HHCM_MISSIONGROUP_H

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

#include <map>
#include <string>
#include <vector>

namespace tree {

class MissionGroup;
/**
 * @brief Singleton registry of all MissionGroups in the tree, and which one is
 * currently active.
 *
 * The registry is populated by groups themselves.
 */
class MissionGroupRegistry
{

public:

    struct State
    {
        std::string stage;
        std::string previous;
        std::string current;
        std::string next;
        std::string cycle_label;
        int cycle = -1;
        int cycle_count = 0;

        bool operator==(const State& other) const;
    };

    static MissionGroupRegistry& instance();

    // declare a group, so it can be tracked
    void declare(const MissionGroup* group);

    // active stack: enter pushes, exit pops
    void enter(const MissionGroup* group);
    void exit(const MissionGroup* group, bool completed);

    // return the current state of the mission, for publishing
    State state() const;

    size_t size() const;

private:

    struct Info
    {
        std::string name;

        const MissionGroup* parent = nullptr;
        size_t index = 0;
        bool is_step = true;
    };

    // resolve the parent/child relationships between groups, so next_of() can work
    void resolve_nesting() const;

    // return the next group at the same level, or nullptr if none
    const MissionGroup* next_of(const MissionGroup* group) const;

    // return the first step of a group, or the group itself if it is a step
    const MissionGroup* first_step(const MissionGroup* group) const;

    mutable std::map<const MissionGroup*, Info> _groups;
    // children of each group, or roots at nullptr
    mutable std::map<const MissionGroup*, std::vector<const MissionGroup*>> _children;
    // groups are declared in order, so the first one is the "stage"
    mutable std::vector<const MissionGroup*> _declared;
    mutable bool _nesting_resolved = false;

    std::vector<const MissionGroup*> _active;
    std::string _last_done;

};

/**
 * @brief Marks a slice of the tree as a named group, so mision progress reads 
 * in human-readable terms (e.g., "Grasp Vial") instead of raw node names.
 *
 *   <MissionGroup name="Uncapping">
 *       <Sequence>
 *           <MissionGroup name="Fold Arm"> ... </MissionGroup>
 *           <MissionGroup name="Uncap Vials"
 *                         cycle="{vial_idx}"
 *                         cycle_count="{num_poses}"
 *                         cycle_label="vial">
 *               <Repeat num_cycles="{num_poses}"> ... </Repeat>
 *           </MissionGroup>
 *       </Sequence>
 *   </MissionGroup>
 *
 * As a decorator it takes one child, so a group wrapping several nodes slips a
 * Sequence in between.
 *
 * Cycle ports are optional and only make sense when the group wraps a loop. The cycle index is 0-based.
 * 
 */
class MissionGroup : public BT::DecoratorNode
{

public:

    MissionGroup(const std::string& name, const BT::NodeConfiguration& config);

    static BT::PortsList providedPorts();

    void halt() override;

    // current iteration, or -1 when the group does not declare one
    int cycle() const;

    // total iterations, or 0 when the group does not wrap a loop
    int cycle_count() const;

    // name of the repeated item, e.g. "vial"
    const std::string& cycle_label() const;

private:

    BT::NodeStatus tick() override;

    void read_cycle_ports();

    bool _active = false;
    int _cycle = -1;
    int _cycle_count = 0;
    std::string _cycle_label;

};

// return a list of nodes that are not wrapped in a MissionGroup
std::vector<std::string> find_ungrouped_nodes(const BT::Tree& tree);

}

#endif // TREE_HHCM_MISSIONGROUP_H
