#include <tree_hhcm/tree_hhcm.h>

#include <tree_hhcm/common/load_pose_from_config.h>
#include <tree_hhcm/common/load_param_from_config.h>
#include <tree_hhcm/common/mission_group.h>

#include <tree_hhcm/ros/node_provider.h>
#include <tree_hhcm/ros/mission_tracker.h>
#include <tree_hhcm/ros/wait_setbool.h>

#include <tree_hhcm/robot/robot_loader.h>
#include <tree_hhcm/robot/robot_move.h>
#include <tree_hhcm/robot/robot_sense.h>
#include <tree_hhcm/robot/robot_gripper_ctrl.h>
#include <tree_hhcm/robot/robot_joint_state_trajectory.h>
#include <tree_hhcm/robot/robot_collision_check.h>
#include <tree_hhcm/robot/robot_set_joint_impedance.h>

#include <tree_hhcm/cartesio/cartesio_cartesian_task_control.h>
#include <tree_hhcm/cartesio/cartesio_loader.h>
#include <tree_hhcm/cartesio/cartesio_solve.h>
#include <tree_hhcm/cartesio/cartesio_tf_publisher.h>

#include <tree_hhcm/anime/anime_log_metrics.h>

BT_REGISTER_NODES(factory)
{
    factory.registerNodeType<tree::LoadPoseFromConfig>("LoadPoseFromConfig");
    factory.registerNodeType<tree::LoadParamFromConfig>("LoadParamFromConfig");
    factory.registerNodeType<tree::MissionGroup>("MissionGroup");

    factory.registerNodeType<tree::NodeProvider>("NodeProvider");
    factory.registerNodeType<tree::MissionTracker>("MissionTracker");
    factory.registerNodeType<tree::WaitSetBool>("WaitSetBool");

    factory.registerNodeType<tree::RobotLoader>("RobotLoader");
    factory.registerNodeType<tree::RobotSense>("RobotSense");
    factory.registerNodeType<tree::RobotMove>("RobotMove");
    factory.registerNodeType<tree::RobotGripperCtrl>("RobotGripperCtrl");
    factory.registerNodeType<tree::RobotJointStateTrajectory>("RobotJointStateTrajectory");
    factory.registerNodeType<tree::RobotCollisionCheck>("RobotCollisionCheck");
    factory.registerNodeType<tree::RobotSetJointImpedance>("RobotSetJointImpedance");
    
    factory.registerNodeType<tree::CartesioLoader>("CartesioLoader");
    factory.registerNodeType<tree::CartesioSolve>("CartesioSolve");
    factory.registerNodeType<tree::CartesioTaskControl>("CartesioTaskControl");
    factory.registerNodeType<tree::CartesioTfPublisher>("CartesioTfPublisher");

    factory.registerNodeType<tree::AnimeLogMetrics>("AnimeLogMetrics");
}

BT::Blackboard::Ptr tree::ConfigValueBase::root_tree_blackboard = nullptr;