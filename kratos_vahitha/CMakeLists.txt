cmake_minimum_required(VERSION 3.8)
project(kratos_vahitha)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------------------
# Interfaces (msg & srv)
# --------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RoverData.msg"
  "srv/CalculateArea.srv"
  "srv/ControlRover.srv"
  DEPENDENCIES geometry_msgs
)

# --------------------
# Install Python scripts
# --------------------
install(
  PROGRAMS
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



