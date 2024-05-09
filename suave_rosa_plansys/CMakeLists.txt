cmake_minimum_required(VERSION 3.8)
project(suave_rosa_plansys)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosa_task_plan_plansys REQUIRED)
find_package(rclcpp REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(rosa_task_plan_plansys REQUIRED)


set(dependencies
  ament_cmake
  mavros_msgs
  rclcpp
  plansys2_executor
  rosa_task_plan_plansys
)

include_directories(
  include
)

add_executable(action_search_pipeline src/action_search_pipeline.cpp)
ament_target_dependencies(action_search_pipeline ${dependencies})

add_executable(action_inspect_pipeline src/action_inspect_pipeline.cpp)
ament_target_dependencies(action_inspect_pipeline ${dependencies})

add_executable(action_start_robot src/action_start_robot.cpp)
ament_target_dependencies(action_start_robot ${dependencies})

add_executable(suave_rosa_controller src/suave_rosa_controller.cpp)
ament_target_dependencies(suave_rosa_controller ${dependencies})


install(TARGETS
  action_search_pipeline
  action_inspect_pipeline
  action_start_robot
  suave_rosa_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  pddl
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
ament_package()
