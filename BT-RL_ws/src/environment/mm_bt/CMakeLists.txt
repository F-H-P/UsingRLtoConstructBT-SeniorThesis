cmake_minimum_required(VERSION 3.8)
project(mm_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)  
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(PROGRAMS
  # scripts/fang_test.py
  scripts/autonomous_modeV2.py
  # scripts/node_test.py
  # scripts/autonomous_modeV2_old.py
  # scripts/behaviours_action_old.py
  # scripts/behaviours_condition_old.py
  # scripts/behaviours_action.py
  # scripts/behaviours_condition.py
  # scripts/test_pub.py
  scripts/test_bt_ja.py
  # scripts/test1.py
  # scripts/respawn.py
  DESTINATION lib/${PROJECT_NAME}
)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   # "srv/CheckDoor.srv"
#   # "srv/MMControl.srv"
#   # "action/MMControl.action"
#   # "action/MMAdjustBase.action"
#   # "action/MoveManipulatorToTarget.action"
#   DEPENDENCIES geometry_msgs nav_msgs# Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
# )

install(
  DIRECTORY 
  # config
  scripts
  DESTINATION share/${PROJECT_NAME}
)

add_executable(my_node src/my_node.cpp)
target_include_directories(my_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(my_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()