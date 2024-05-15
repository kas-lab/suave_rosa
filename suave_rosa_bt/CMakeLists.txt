cmake_minimum_required(VERSION 3.8)
project(suave_rosa_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosa_msgs REQUIRED)
find_package(rosa_task_plan_bt REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(suave_bt REQUIRED)

set(dependencies
  ament_index_cpp
  behaviortree_cpp
  rclcpp
  rosa_msgs
  suave_bt
  rosa_task_plan_bt
  std_msgs
  std_srvs
)

include_directories(
  include
)

add_library(action_search_pipeline_bt_node SHARED src/suave_rosa_bt/action_search_pipeline.cpp)
add_library(action_inspect_pipeline_bt_node SHARED src/suave_rosa_bt/action_inspect_pipeline.cpp)
add_library(action_recharge_battery_bt_node SHARED src/suave_rosa_bt/action_recharge_battery.cpp)

list(APPEND plugin_libs
  action_search_pipeline_bt_node
  action_inspect_pipeline_bt_node
  action_recharge_battery_bt_node
)

foreach(bt_plugin ${plugin_libs})
  target_compile_features(${bt_plugin} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
  target_include_directories(${bt_plugin} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE "SUAVE_ROSA_BT_BUILDING_LIBRARY")
endforeach()

add_executable(suave_rosa_bt src/suave_rosa_bt.cpp)
ament_target_dependencies(suave_rosa_bt ${dependencies})
target_link_libraries(suave_rosa_bt ${plugin_libs})

add_executable(suave_rosa_extended_bt src/suave_rosa_bt_extended.cpp)
ament_target_dependencies(suave_rosa_extended_bt ${dependencies})
target_link_libraries(suave_rosa_extended_bt ${plugin_libs})

install(TARGETS
  suave_rosa_bt
  suave_rosa_extended_bt
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# add config, launch etc, whatever is needed
install(DIRECTORY
  bts
  launch
  DESTINATION share/${PROJECT_NAME}
)

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

ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_package()
