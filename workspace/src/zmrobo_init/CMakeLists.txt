cmake_minimum_required(VERSION 3.10)
project(zmrobo_init)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# tf2_geometry_msgs (ROS2 Foxy)
find_package(tf2_geometry_msgs REQUIRED)
# serial (ROS2 Foxy package)
find_package(serial REQUIRED)

# ulimit -c unlimited
# ssh firefly@192.168.1.76
add_compile_options(-g -std=c++17)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${PCL_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME})

add_executable(zmrobo_node src/zmrobo.cpp src/Quaternion_Solution.cpp)
ament_target_dependencies(zmrobo_node rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs ackermann_msgs tf2 tf2_ros tf2_geometry_msgs serial)

install(TARGETS zmrobo_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
