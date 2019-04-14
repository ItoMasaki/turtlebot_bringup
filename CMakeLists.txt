cmake_minimum_required(VERSION 3.5)
project(turtlebot_bringup)

# Default to C++14
set(CMAKE_CXX_STANDARD 14)

add_compile_options(-Wall -Wextra -Wpedantic -Wunused-variable -Wunused-parameter -Wpedantic)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ./lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ./bin)

add_library(DockingController SHARED src/DockingController.cpp)
add_library(Kobuki_impl       SHARED src/Kobuki_impl.cpp src/Thread.cpp src/KobukiBase.cpp src/DockingController.cpp src/SerialPort.cpp src/Translator.cpp src/Transport.cpp)
add_library(Thread            SHARED src/Thread.cpp)
add_library(Transport         SHARED src/Transport.cpp src/SerialPort.cpp)
add_library(kobuki            SHARED src/libkobuki.cpp src/Kobuki_impl.cpp src/Thread.cpp src/KobukiBase.cpp src/DockingController.cpp src/SerialPort.cpp src/Translator.cpp src/Transport.cpp)
add_library(KobukiBase        SHARED src/KobukiBase.cpp src/SerialPort.cpp src/Thread.cpp src/Transport.cpp src/Translator.cpp)
add_library(SerialPort        SHARED src/SerialPort.cpp)
add_library(Translator        SHARED src/Translator.cpp)
add_library(kobukicwrapper    SHARED src/kobukicwrapper.cpp src/Kobuki_impl.cpp src/Thread.cpp src/libkobuki.cpp src/DockingController.cpp src/KobukiBase.cpp src/SerialPort.cpp src/Transport.cpp src/Translator.cpp)

add_executable(turtlebot2 src/turtlebot2.cpp src/DockingController.cpp src/Kobuki_impl.cpp src/Thread.cpp src/Transport.cpp src/libkobuki.cpp src/KobukiBase.cpp src/SerialPort.cpp src/Translator.cpp)

ament_target_dependencies(turtlebot2 rclcpp std_msgs geometry_msgs lib/DockingController lib/Kobuki_impl lib/Thread lib/Transport lib/kobuki lib/KobukiBase lib/SerialPort lib/Translator lib/kobukicwrapper)

target_include_directories(turtlebot2 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS turtlebot2
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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
