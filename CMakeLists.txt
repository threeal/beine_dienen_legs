cmake_minimum_required(VERSION 3.5)

project(bein_bridge)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(bein_interfaces REQUIRED)
find_package(musen REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME}
  "src/listeners/leg_listener.cpp"
  "src/listeners/voice_listener.cpp"
  "src/bridge.cpp")

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} bein_interfaces musen rclcpp)

install(DIRECTORY "include" DESTINATION ".")

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  RUNTIME DESTINATION "bin")

add_executable(bridge "bin/bridge.cpp")

target_include_directories(bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_link_libraries(bridge ${PROJECT_NAME})

install(TARGETS bridge
  DESTINATION "lib/${PROJECT_NAME}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(bein_interfaces musen rclcpp)
ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_package()
