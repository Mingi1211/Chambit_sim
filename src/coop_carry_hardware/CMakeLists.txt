cmake_minimum_required(VERSION 3.16)
project(coop_carry_hardware)

# ROS 제어 인터페이스와 Gazebo 플러그인 의존성
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)

# 로봇 컨테이너: Controller Manager가 불러오는 원격 하드웨어
add_library(coop_carry_remote_hardware SHARED src/remote_gazebo_system.cpp)
target_compile_features(coop_carry_remote_hardware PUBLIC cxx_std_17)
ament_target_dependencies(coop_carry_remote_hardware
  hardware_interface
  pluginlib
  rclcpp
  sensor_msgs
)
pluginlib_export_plugin_description_file(hardware_interface hardware_plugins.xml)

add_library(coop_carry_remote_swerve SHARED src/remote_swerve_system.cpp)
target_compile_features(coop_carry_remote_swerve PUBLIC cxx_std_17)
ament_target_dependencies(coop_carry_remote_swerve hardware_interface pluginlib rclcpp sensor_msgs actuator_msgs)
pluginlib_export_plugin_description_file(hardware_interface swerve_plugins.xml)

add_library(coop_carry_gazebo_swerve SHARED src/gazebo_swerve_actuator.cpp)
target_compile_features(coop_carry_gazebo_swerve PUBLIC cxx_std_17)
target_link_libraries(coop_carry_gazebo_swerve gz-sim8::gz-sim8 gz-plugin2::register)

# Gazebo 컨테이너: 관절 구동 플러그인
add_library(coop_carry_gazebo_actuator SHARED src/gazebo_joint_actuator.cpp)
target_compile_features(coop_carry_gazebo_actuator PUBLIC cxx_std_17)
target_link_libraries(coop_carry_gazebo_actuator gz-sim8::gz-sim8 gz-plugin2::register)

# Gazebo 서버 실행 및 종료 처리
add_executable(coop_gazebo_server src/gazebo_server.cpp)
target_compile_features(coop_gazebo_server PUBLIC cxx_std_17)
ament_target_dependencies(coop_gazebo_server rclcpp)
target_link_libraries(coop_gazebo_server gz-sim8::gz-sim8)

install(TARGETS coop_gazebo_server RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS
  coop_carry_remote_hardware
  coop_carry_gazebo_actuator
  coop_carry_remote_swerve
  coop_carry_gazebo_swerve
  LIBRARY DESTINATION lib
)

ament_package()
