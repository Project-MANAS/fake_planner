cmake_minimum_required(VERSION 3.5)
project(fake_planner)

set(CMAKE_CXX_FLAGS "-std=c++17 -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        message_generation
)

add_service_files(
        FILES
        SetMaxVel.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(CATKIN_DEPENDS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        message_runtime
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(fake_planner src/fake_planner_node.cpp src/FakePlanner.cpp)
target_link_libraries(fake_planner ${catkin_LIBRARIES})
add_dependencies(fake_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
