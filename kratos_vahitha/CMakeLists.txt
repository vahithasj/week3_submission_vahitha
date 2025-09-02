cmake_minimum_required(VERSION 3.8)
project(kratos_vahitha)

# Default C++ standard (if you use C++ later, safe to keep)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add message and service definitions
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverData.msg"
  "srv/CalculateArea.srv"
  "srv/ControlRover.srv"
  DEPENDENCIES geometry_msgs


)

ament_export_dependencies(rosidl_default_runtime)

# Install Python executables (scripts)
install(PROGRAMS
  scripts/q1_fk.py
  scripts/q1_pub.py
  scripts/q1_sub.py
  scripts/q2_s1.py
  scripts/q2_s2.py
  scripts/q3_rover.py
  scripts/q4_server.py
  scripts/q4_client.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


