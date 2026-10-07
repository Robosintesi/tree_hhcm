#include <tree_hhcm/common/string_format.h>

namespace tree {

namespace {

const std::string PLACEHOLDER = "{i}";

std::string substitute(std::string str, const std::string &value)
{
    for(size_t at = str.find(PLACEHOLDER);
        at != std::string::npos;
        at = str.find(PLACEHOLDER, at + value.size()))
    {
        str.replace(at, PLACEHOLDER.size(), value);
    }

    return str;
}

}

StringFormat::StringFormat(std::string name, const BT::NodeConfiguration &config):
    BT::SyncActionNode(name, config), _p(*this)
{
}

BT::NodeStatus StringFormat::tick()
{
    std::string format;

    if(!getInput("format", format))
    {
        throw BT::RuntimeError("StringFormat: missing required input [format]");
    }

    int idx = getInput<int>("idx").value_or(0);

    setOutput<std::string>("value", substitute(format, std::to_string(idx)));

    return BT::NodeStatus::SUCCESS;
}

BT::PortsList StringFormat::providedPorts()
{
    return {
        BT::InputPort<std::string>("format", "Template string; every {i} in it is replaced by idx"),
        BT::InputPort<int>("idx", 0, "Index substituted into the template, starting from 0"),
        BT::OutputPort<std::string>("value", "The template with {i} substituted")
    };
}

} // namespace tree
