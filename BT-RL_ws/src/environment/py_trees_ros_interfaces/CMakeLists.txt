##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(py_trees_ros_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

##############################################################################
# Dependencies
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unique_identifier_msgs REQUIRED)

##############################################################################
# Messages
##############################################################################

set(action_files
  "action/Dock.action"
  "action/Rotate.action"
  "action/MoveBase.action"
)

set(msg_files
  "msg/ActivityItem.msg"
  "msg/Behaviour.msg"
  "msg/BehaviourTree.msg"
  "msg/KeyValue.msg"
  "msg/PublisherDetails.msg"
  "msg/Statistics.msg"
  "msg/ServiceDetails.msg"
  "msg/SnapshotStreamParameters.msg"
  "msg/SubscriberDetails.msg"
)

set(srv_files
  "srv/CloseBlackboardStream.srv"
  "srv/CloseSnapshotStream.srv"
  "srv/GetBlackboardVariables.srv"
  "srv/IntrospectPublishers.srv"
  "srv/IntrospectServices.srv"
  "srv/IntrospectSubscribers.srv"
  "srv/OpenBlackboardStream.srv"
  "srv/OpenSnapshotStream.srv"
  "srv/ReconfigureSnapshotStream.srv"
  "srv/StatusReport.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  ${action_files}
  DEPENDENCIES
    action_msgs  # even though not directly using, it requires some trace back
    diagnostic_msgs
    geometry_msgs
    unique_identifier_msgs
  # ADD_LINTER_TESTS  # very expensive
)
##############################################################################
# Package
##############################################################################

ament_export_dependencies(rosidl_default_runtime)

ament_package()
