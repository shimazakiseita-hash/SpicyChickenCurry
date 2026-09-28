cmake_minimum_required(VERSION 3.5)
project(crane_x7_examples)

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
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_geometry REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core)
# Lyrical (VTK 9.5): VTK::jsoncpp が参照する JsonCpp::JsonCpp を先に定義しておく
find_package(jsoncpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build and install node executables
set(executable_list
  gripper_control
  cartesian_path
  pose_groupstate
  joint_values
  pick_and_place
  pick_and_place_tf
  aruco_detection
  color_detection
  point_cloud_detection
)
foreach(loop_var IN LISTS executable_list)
  add_executable(${loop_var} src/${loop_var}.cpp)
  target_link_libraries(${loop_var}
    ${cv_bridge_TARGETS}
    ${geometry_msgs_TARGETS}
    ${image_geometry_TARGETS}
    ${image_transport_TARGETS}
    ${message_filters_TARGETS}
    ${moveit_ros_planning_interface_TARGETS}
    ${OpenCV_LIBS}
    ${pcl_ros_TARGETS}
    ${pcl_conversions_TARGETS}
    ${rclcpp_TARGETS}
    ${tf2_geometry_msgs_TARGETS}
  )

  install(TARGETS
    ${loop_var}
    DESTINATION lib/${PROJECT_NAME}
  )
endforeach()

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
