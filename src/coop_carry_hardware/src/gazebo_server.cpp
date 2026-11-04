#include <exception>
#include <iostream>
#include <stdexcept>

#include "gz/common/SignalHandler.hh"
#include "gz/sim/Server.hh"
#include "gz/sim/ServerConfig.hh"
#include "rclcpp/rclcpp.hpp"

// Own the server lifetime in main so ROS cleanup happens before process-exit
// destructors destroy Tokio TLS. The upstream control plugins still initialize
// ROS themselves, preserving their arguments and shared default context.
int main(int argc, char ** argv)
{
  if (argc != 2) {
    std::cerr << "Usage: coop_gazebo_server WORLD.sdf\n";
    return 2;
  }
  int result = 0;
  try {
    gz::sim::ServerConfig config;
    if (!config.SetSdfFile(argv[1])) {
      throw std::runtime_error("Could not set Gazebo world file");
    }
    {
      gz::sim::Server server(config);
      // An embedded Server needs its own signal-to-Stop callback when no
      // gz_ros2_control plugin initializes ROS in the server process.
      gz::common::SignalHandler signals;
      if (!signals.AddCallback([&server](int) { server.Stop(); })) {
        throw std::runtime_error("Could not install Gazebo shutdown signal handler");
      }
      if (!server.Run(true, 0, false)) {
        std::cerr << "Gazebo server failed to run\n";
        result = 1;
      }
      std::cout << "Simulation stopped; unloading Gazebo systems" << std::endl;
    }
    std::cout << "Gazebo server stopped cleanly" << std::endl;
  } catch (const std::exception & error) {
    std::cerr << "Gazebo server: " << error.what() << '\n';
    result = 1;
  }
  try {
    // All Gazebo models and their controller executors have been destroyed.
    // Shut down the shared ROS context once, while main's TLS is still alive.
    if (rclcpp::ok()) {
      rclcpp::shutdown();
      std::cout << "Gazebo systems destroyed; ROS context shut down\n";
    }
  } catch (const std::exception & error) {
    std::cerr << "ROS shutdown: " << error.what() << '\n';
    result = 1;
  }
  return result;
}
