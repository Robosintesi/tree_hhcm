cmake_minimum_required(VERSION 3.5)
project(tree_hhcm VERSION 0.0.1)

include(cmake/ExportProject.cmake)
include(GNUInstallDirs)

set(CMAKE_CXX_STANDARD 20)

find_package(behaviortree_cpp REQUIRED)
find_package(cartesian_interface REQUIRED)
find_package(cartesian_interface_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xbot_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)

# the library
add_library(tree_hhcm SHARED 
  src/parsers.cpp
  src/cartesio_solve.cpp
  src/cartesio_loader.cpp
  src/cartesio_cartesian_task_control.cpp
  src/cartesio_tf_publisher.cpp
  src/robot_loader.cpp
  src/robot_sense.cpp
  src/robot_move.cpp
  src/robot_joint_state_trajectory.cpp
  src/robot_play_trajectory.cpp
  src/robot_gripper_ctrl.cpp
  src/robot_collision_check.cpp
  src/robot_set_joint_impedance.cpp
  src/node_provider.cpp
  src/wait_setbool.cpp
  src/pause_server.cpp
  src/mission_group.cpp
  src/mission_tracker.cpp
  src/tree_hhcm.cpp
  src/common.cpp
  src/load_pose_from_config.cpp
  src/load_param_from_config.cpp
  src/string_format.cpp
  src/anime_log_metrics.cpp
  )

target_link_libraries(tree_hhcm 
    PUBLIC 
    behaviortree_cpp::behaviortree_cpp 
    cartesian_interface::CartesianInterface
    cartesian_interface_ros::CartesianInterfaceRos
    rclcpp::rclcpp
    ${xbot_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${std_msgs_TARGETS}
    )

target_include_directories(tree_hhcm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

target_compile_definitions(tree_hhcm PRIVATE  BT_PLUGIN_EXPORT )

# the executable
add_executable(tree_hhcm_executor src/tree_hhcm_main.cpp)

target_link_libraries(tree_hhcm_executor
    PRIVATE
    tree_hhcm
    )

install(
    TARGETS 
    tree_hhcm 
    tree_hhcm_executor
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}
    )

export_project()
