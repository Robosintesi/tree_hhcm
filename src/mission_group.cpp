#include <tree_hhcm/common/mission_group.h>

#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/decorator_node.h>

#include <algorithm>

using namespace tree;


bool MissionGroupRegistry::State::operator==(const State& other) const
{
    return stage == other.stage &&
           previous == other.previous &&
           current == other.current &&
           next == other.next &&
           cycle_label == other.cycle_label &&
           cycle == other.cycle &&
           cycle_count == other.cycle_count;
}

MissionGroupRegistry& MissionGroupRegistry::instance()
{
    static MissionGroupRegistry reg;
    return reg;
}

void MissionGroupRegistry::declare(const MissionGroup* group)
{
    Info info;
    info.name = group->name();

    _groups[group] = std::move(info);
    _declared.push_back(group);
    _nesting_resolved = false;
}

namespace {

void nearest_groups(const BT::TreeNode* node,
                    std::vector<const MissionGroup*>& out)
{
    if(node == nullptr)
    {
        return;
    }

    if(auto group = dynamic_cast<const MissionGroup*>(node))
    {
        out.push_back(group);
        return;
    }

    if(auto control = dynamic_cast<const BT::ControlNode*>(node))
    {
        for(const auto* child : control->children())
        {
            nearest_groups(child, out);
        }
    }
    else if(auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
    {
        nearest_groups(decorator->child(), out);
    }
}

}

void MissionGroupRegistry::resolve_nesting() const
{
    if(_nesting_resolved)
    {
        return;
    }

    _children.clear();

    for(auto& [group, info] : _groups)
    {
        info.parent = nullptr;
    }

    // nobody can find their own parent, so each group walks its own subtree and
    // writes parent and index onto the children it finds
    for(const auto* group : _declared)
    {
        auto& children = _children[group];

        nearest_groups(group->child(), children);

        for(size_t i = 0; i < children.size(); ++i)
        {
            auto& child_info = _groups.at(children[i]);
            child_info.parent = group;
            child_info.index = i;
        }
    }

    // whatever nobody claimed is a root
    auto& roots = _children[nullptr];

    for(const auto* group : _declared)
    {
        if(_groups.at(group).parent == nullptr)
        {
            _groups.at(group).index = roots.size();
            roots.push_back(group);
        }
    }

    // a group with no groups inside is a step
    for(const auto* group : _declared)
    {
        _groups.at(group).is_step = _children[group].empty();
    }

    _nesting_resolved = true;
}

void MissionGroupRegistry::enter(const MissionGroup* group)
{
    resolve_nesting();

    _active.push_back(group);
}

void MissionGroupRegistry::exit(const MissionGroup* group, bool completed)
{
    auto it = std::find(_active.begin(), _active.end(), group);

    if(it == _active.end())
    {
        return;
    }

    // anything above it in the stack is over too
    _active.erase(it, _active.end());

    if(!completed)
    {
        return;
    }

    const auto& info = _groups.at(group);

    if(info.is_step)
    {
        _last_done = info.name;
    }
}

size_t MissionGroupRegistry::size() const
{
    return _groups.size();
}

const MissionGroup* MissionGroupRegistry::next_of(const MissionGroup* group) const
{
    const auto& info = _groups.at(group);

    if(info.parent == nullptr)
    {
        return nullptr;
    }

    const auto& siblings = _children.at(info.parent);

    if(info.index + 1 < siblings.size())
    {
        return siblings[info.index + 1];
    }

    const int count = info.parent->cycle_count();

    if(count > 0 && info.parent->cycle() + 1 < count)
    {
        return siblings.front();
    }

    return next_of(info.parent);
}

// descend a container down to the step it will actually show as current
const MissionGroup* MissionGroupRegistry::first_step(const MissionGroup* group) const
{
    // is_step means no groups inside, so a container always has children
    while(!_groups.at(group).is_step)
    {
        group = _children.at(group).front();
    }

    return group;
}

MissionGroupRegistry::State MissionGroupRegistry::state() const
{
    State s;

    if(_active.empty())
    {
        return s;
    }

    resolve_nesting();

    const auto* innermost = _active.back();
    const auto& info = _groups.at(innermost);

    s.current = info.name;
    s.stage = _groups.at(_active.front()).name;
    s.previous = _last_done;

    if(const auto* next = next_of(innermost))
    {
        s.next = _groups.at(first_step(next)).name;
    }

    for(const auto* group : _active)
    {
        if(group->cycle_count() > 0)
        {
            s.cycle = group->cycle();
            s.cycle_count = group->cycle_count();
            s.cycle_label = group->cycle_label();
            break;
        }
    }

    return s;
}


MissionGroup::MissionGroup(const std::string& name, const BT::NodeConfiguration& config):
    BT::DecoratorNode(name, config)
{
    MissionGroupRegistry::instance().declare(this);
}

BT::PortsList MissionGroup::providedPorts()
{
    return {
        BT::InputPort<int>("cycle", "current iteration of the loop this group wraps, 0-based"),
        BT::InputPort<int>("cycle_count", "total number of iterations of that loop"),
        BT::InputPort<std::string>("cycle_label", "name of the repeated item, e.g. 'vial'"),
    };
}

int MissionGroup::cycle() const
{
    return _cycle;
}

int MissionGroup::cycle_count() const
{
    return _cycle_count;
}

const std::string& MissionGroup::cycle_label() const
{
    return _cycle_label;
}

void MissionGroup::read_cycle_ports()
{
    if(auto v = getInput<int>("cycle"))
    {
        _cycle = v.value();
    }

    if(auto v = getInput<int>("cycle_count"))
    {
        _cycle_count = v.value();
    }

    if(auto v = getInput<std::string>("cycle_label"))
    {
        _cycle_label = v.value();
    }
}

BT::NodeStatus MissionGroup::tick()
{
    read_cycle_ports();

    if(!_active)
    {
        _active = true;
        MissionGroupRegistry::instance().enter(this);
    }

    setStatus(BT::NodeStatus::RUNNING);

    const BT::NodeStatus child_status = child_node_->executeTick();

    if(child_status == BT::NodeStatus::SKIPPED)
    {
        resetChild();

        _active = false;
        MissionGroupRegistry::instance().exit(this, false);

        return child_status;
    }

    if(isStatusCompleted(child_status))
    {
        resetChild();

        _active = false;
        MissionGroupRegistry::instance().exit(this, true);
    }

    return child_status;
}

void MissionGroup::halt()
{
    if(_active)
    {
        _active = false;
        MissionGroupRegistry::instance().exit(this, false);
    }

    BT::DecoratorNode::halt();
}


namespace {

bool is_composite(const BT::TreeNode* node)
{
    return dynamic_cast<const BT::ControlNode*>(node) != nullptr ||
           dynamic_cast<const BT::DecoratorNode*>(node) != nullptr;
}

void collect_ungrouped(const BT::TreeNode* node, std::vector<std::string>& out)
{
    if(node == nullptr)
    {
        return;
    }

    if(auto control = dynamic_cast<const BT::ControlNode*>(node))
    {
        const auto& children = control->children();

        const bool any_group = std::any_of(
            children.begin(), children.end(),
            [](const BT::TreeNode* c) { return dynamic_cast<const MissionGroup*>(c) != nullptr; });

        for(const auto* child : children)
        {
            if(any_group &&
               is_composite(child) &&
               dynamic_cast<const MissionGroup*>(child) == nullptr)
            {
                out.push_back(child->fullPath());
            }

            collect_ungrouped(child, out);
        }
    }
    else if(auto decorator = dynamic_cast<const BT::DecoratorNode*>(node))
    {
        collect_ungrouped(decorator->child(), out);
    }
}

}

std::vector<std::string> tree::find_ungrouped_nodes(const BT::Tree& tree)
{
    std::vector<std::string> out;

    collect_ungrouped(tree.rootNode(), out);

    return out;
}
