cmake_minimum_required(VERSION 3.5)

project(turtlebot_bringup)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-O3 -Wall -Wextra -Wpedantic -Wunused-variable -Wunused-parameter -Wpedantic -Wdeprecated-declarations)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

include_directories(include src/bringup)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

add_library(DockingController SHARED kobuki/DockingController.cpp)
add_library(Kobuki_impl       SHARED kobuki/Kobuki_impl.cpp kobuki/Thread.cpp kobuki/KobukiBase.cpp kobuki/DockingController.cpp kobuki/SerialPort.cpp kobuki/Translator.cpp kobuki/Transport.cpp)
add_library(Thread            SHARED kobuki/Thread.cpp)
add_library(Transport         SHARED kobuki/Transport.cpp kobuki/SerialPort.cpp)
add_library(kobuki            SHARED kobuki/libkobuki.cpp kobuki/Kobuki_impl.cpp kobuki/Thread.cpp kobuki/KobukiBase.cpp kobuki/DockingController.cpp kobuki/SerialPort.cpp kobuki/Translator.cpp kobuki/Transport.cpp)
add_library(KobukiBase        SHARED kobuki/KobukiBase.cpp kobuki/SerialPort.cpp kobuki/Thread.cpp kobuki/Transport.cpp kobuki/Translator.cpp)
add_library(SerialPort        SHARED kobuki/SerialPort.cpp)
add_library(Translator        SHARED kobuki/Translator.cpp)
add_library(kobukicwrapper    SHARED kobuki/kobukicwrapper.cpp kobuki/Kobuki_impl.cpp kobuki/Thread.cpp kobuki/libkobuki.cpp kobuki/DockingController.cpp kobuki/KobukiBase.cpp kobuki/SerialPort.cpp kobuki/Transport.cpp kobuki/Translator.cpp)

add_library(Turtlebot		  SHARED src/bringup/Turtlebot.cpp)
ament_target_dependencies(Turtlebot rclcpp tf2_ros)

add_executable(turtlebot2 src/bringup/turtlebot2.cpp src/bringup/Turtlebot.cpp kobuki/DockingController.cpp kobuki/Kobuki_impl.cpp kobuki/Thread.cpp kobuki/Transport.cpp kobuki/libkobuki.cpp kobuki/KobukiBase.cpp kobuki/SerialPort.cpp kobuki/Translator.cpp)

add_executable(teleop_node src/teleop/teleop_node.cpp)

ament_target_dependencies(turtlebot2 rclcpp tf2_ros std_msgs geometry_msgs nav_msgs sensor_msgs)

ament_target_dependencies(teleop_node rclcpp sensor_msgs geometry_msgs)

target_include_directories(turtlebot2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS turtlebot2
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS teleop_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
