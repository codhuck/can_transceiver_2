cmake_minimum_required(VERSION 3.16)
project(can_transceiver_2 LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  std_msgs
  rclcpp
  realtime_tools
  rcpputils
  pluginlib
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(include/)

add_executable(${PROJECT_NAME}
  src/can_transceiver.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  std_msgs
  rclcpp
  rcpputils
  realtime_tools
  pluginlib
)

# INSTALL
install(TARGETS 
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})


ament_package()