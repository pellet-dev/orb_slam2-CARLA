cmake_minimum_required(VERSION 3.5)
project(orb_node)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(OpenCV 3.0 REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(Eigen3 3.2.10 REQUIRED NO_MODULE)
find_package(Pangolin REQUIRED NO_MODULE)

add_executable(stereo-node src/stereo-node.cpp)
ament_target_dependencies(stereo-node rclcpp std_msgs nav_msgs message_filters Pangolin tf2_ros tf2)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${cv_bridge_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR} ${Pangolin_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/src/ORB_SLAM2)

target_link_libraries(stereo-node ${OpenCV_LIBS} ${cv_bridge_LIBRARIES} ${message_filters_LIBRARIES} ${OpenCV_LIBS} ${Pangolin_LIBRARIES}
${EIGEN3_LIBS} ${PROJECT_SOURCE_DIR}/src/ORB_SLAM2/lib/libORB_SLAM2.so)


install(TARGETS
  stereo-node
  DESTINATION lib/${PROJECT_NAME})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()