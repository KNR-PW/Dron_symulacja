cmake_minimum_required(VERSION 3.8)
project(drone_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoMarkers.msg"
  "msg/MiddleOfAruco.msg"
  "msg/VelocityVectors.msg"
  "msg/AxisAngle.msg"
  "msg/Telemetry.msg"
  "srv/Dropper.srv"
  "srv/GetAttitude.srv"
  "srv/GetLocationRelative.srv"
  "srv/SetYaw.srv"
  "srv/SetServo.srv"
  "srv/SetMode.srv"
  "srv/SetSpeed.srv"
  "srv/GetGpsPos.srv"
  "srv/ToggleVelocityControl.srv"
  "srv/SpinDCMotor.srv"
  "srv/PostLog.srv"
  "srv/TurnOnVideo.srv"
  "srv/TurnOffVideo.srv"
  "srv/CreateReport.srv"
  "srv/UpdateReport.srv"
  "action/GotoRelative.action"
  "action/GotoGlobal.action"
  "action/Arm.action"
  "action/SetYawAction.action"
  "action/Takeoff.action"
  "srv/TakePhoto.srv"
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
 )
ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
