cmake_minimum_required(VERSION 3.8)
project(riptide_rviz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 COMPONENTS Widgets Core REQUIRED)
find_package(pluginlib REQUIRED)

# Qt5 boilerplate options from http://doc.qt.io/qt-5/cmake-manual.html
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

# pick up headerfiles
include_directories(include)

# add all of the source files here
add_library(${PROJECT_NAME} SHARED
            src/control_panel.cpp
)

# Link ament packages
ament_target_dependencies(${PROJECT_NAME} rclcpp rviz_common rclcpp_action std_srvs interaction_msgs)

# Link non ament packages
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

# install the plugin into lib
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

# export button icons for the plugin
install(DIRECTORY data
 DESTINATION share/${PROJECT_NAME})


# export the plugin icons shown by RVIZ
install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}")


pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

ament_export_libraries(${PROJECT_NAME})

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
