cmake_minimum_required(VERSION 3.5)
project(galaxy_camera_u3v)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include SYSTEM)

add_library(camera_components SHARED
  src/u3v_image_pub.cpp
  src/capture_trigger.cpp
  src/stereo_image_viz.cpp
  src/calibration_stereo_image_capture.cpp
  src/stereo_frame_capture.cpp
)

target_compile_definitions(camera_components
  PRIVATE "CAMERA_BUILDING_DLL")
ament_target_dependencies(camera_components
  rclcpp
  rclcpp_components
  sensor_msgs
  image_transport
  camera_info_manager
  std_msgs
  OpenCV)

target_link_libraries(camera_components
  pthread
  gxiapi
  ${OpenCV_LIBRARIES}
)

rclcpp_components_register_node(camera_components PLUGIN "camera::U3vImagePub" EXECUTABLE u3v_image_pub)
rclcpp_components_register_node(camera_components PLUGIN "camera::CaptureTrigger" EXECUTABLE capture_trigger)
rclcpp_components_register_node(camera_components PLUGIN "camera::StereoImageViz" EXECUTABLE stereo_image_viz)
rclcpp_components_register_node(camera_components PLUGIN "camera::CalibStereoImageCap" EXECUTABLE calib_stereo_image_cap)
rclcpp_components_register_node(camera_components PLUGIN "stereo_capture::StereoFrameCap" EXECUTABLE calib_frame_capture)

install(TARGETS
  camera_components
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY launch camera_info
  DESTINATION share/${PROJECT_NAME}
)

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
