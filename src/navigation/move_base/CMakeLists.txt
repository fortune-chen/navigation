cmake_minimum_required(VERSION 3.0.2)

project(move_base)

aux_source_directory(./src SRC)

# add_library(move_base SHARED ${SRC})

add_executable(move_base_node test/move_base_node.cpp src/move_base.cpp)

if (TF2_BRIDGE)
    target_link_libraries(move_base_node boost_system pthread tf2_bridge costmap_2d orocos-kdl clear_costmap_recovery global_planner dwa_local_planner tinyxml tinyxml2 )
else()
    target_link_libraries(move_base_node boost_system pthread transformer costmap_2d full_coverage_path_planner orocos-kdl clear_costmap_recovery global_planner dwa_local_planner tinyxml tinyxml2 tracking_pid)
endif()


# target_link_libraries(move_base_node boost_system pthread rostime ros costmap_2d orocos-kdl robot_state_publisher clear_costmap_recovery global_planner dwa_local_planner)