cmake_minimum_required(VERSION 3.0.2)
project(multi_map_nav)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  actionlib
  actionlib_msgs
  genmsg
  move_base_msgs
  tf
  std_srvs
  nav_msgs
  rospack
)


add_action_files(
  DIRECTORY action FILES Navigation.action
)


generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    std_msgs
)


catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    actionlib
    actionlib_msgs
    move_base_msgs
    std_srvs
    nav_msgs
    rospack
)

link_directories(/usr/lib)

add_executable(navigation_server src/navigation_server.cpp)
target_link_libraries(navigation_server
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  sqlite3
)
add_dependencies(navigation_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}

)
