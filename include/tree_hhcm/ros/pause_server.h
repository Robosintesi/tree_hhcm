#ifndef _tree_hhcm_PAUSESERVER_H
#define _tree_hhcm_PAUSESERVER_H

#include <tree_hhcm/common/common.h>

// ros
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>

namespace tree {

/**
 * @brief Exposes a ROS2 service to pause/resume the mission.
 *
 * The executor main loop calls paused() once per cycle; while paused,
 * the loop must skip tree.tickOnce(). Since stateful nodes advance
 * their internal time by tree_dt per tick, freezing the ticks freezes
 * all trajectories in place, and the mission resumes exactly where it
 * stopped.
 *
 * Interface (given executor name "tree_main"):
 *   ros2 service call /tree_main/pause std_srvs/srv/SetBool "{data: true}"   # pause
 *   ros2 service call /tree_main/pause std_srvs/srv/SetBool "{data: false}"  # resume
 *   ros2 topic echo /tree_main/paused   # latched state
 */
class PauseServer
{

public:

    PauseServer(std::string name);

    // process pending requests, return true if the mission is paused
    bool paused();

private:

    void set_paused(bool paused);

    rclcpp::Node::SharedPtr _node;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr _srv;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr _pub;
    bool _paused = false;
    Printer _p;

};

}

#endif // _tree_hhcm_PAUSESERVER_H
