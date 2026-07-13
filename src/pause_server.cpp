#include <tree_hhcm/ros/pause_server.h>

using namespace tree;

PauseServer::PauseServer(std::string name):
    _p(name + "_pause_server")
{
    _node = rclcpp::Node::make_shared(name + "_pause_server");

    _pub = _node->create_publisher<std_msgs::msg::Bool>(
        "/" + name + "/paused",
        rclcpp::QoS(1).transient_local());

    _srv = _node->create_service<std_srvs::srv::SetBool>(
        "/" + name + "/pause",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
               std::shared_ptr<std_srvs::srv::SetBool::Response> response)
        {
            set_paused(request->data);
            response->success = true;
            response->message = _paused ? "mission paused" : "mission resumed";
        });

    std_msgs::msg::Bool msg;
    msg.data = _paused;
    _pub->publish(msg);

    _p.cout() << "pause service available on [" << _srv->get_service_name() << "]\n";
}

bool PauseServer::paused()
{
    rclcpp::spin_some(_node);

    return _paused;
}

void PauseServer::set_paused(bool paused)
{
    if(paused == _paused)
    {
        return;
    }

    _paused = paused;

    _p.cout() << (_paused ? "mission paused" : "mission resumed") << "\n";

    std_msgs::msg::Bool msg;
    msg.data = _paused;
    _pub->publish(msg);
}
