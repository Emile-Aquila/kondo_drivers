cmake_minimum_required(VERSION 3.8)
project(kondo_drivers)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()  # packageから自動でfind_packageしてくれる


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


# msg/srv generation
file(GLOB ${PROJECT_NAME}_msg_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*)
file(GLOB ${PROJECT_NAME}_srv_files RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} srv/*)
rosidl_generate_interfaces(
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_msg_files}
        ${${PROJECT_NAME}_srv_files}
        DEPENDENCIES ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
)


# Component Settings
## kondo_driver_node_component
ament_auto_add_library(kondo_driver_node_component SHARED src/kondo_driver_node.cpp src/b3m_commands.cpp)
rclcpp_components_register_nodes(kondo_driver_node_component "kondo_drivers::KondoB3mDriverNode")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(kondo_driver_node_component "${cpp_typesupport_target}")

## kondo_driver_service_component
ament_auto_add_library(kondo_driver_service_component SHARED src/kondo_driver_service.cpp src/b3m_commands.cpp)
rclcpp_components_register_nodes(kondo_driver_service_component "kondo_drivers::KondoB3mDriverService")
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(kondo_driver_service_component "${cpp_typesupport_target}")


# package setting
ament_auto_package(
  INSTALL_TO_SHARE
    launch
    params
)

