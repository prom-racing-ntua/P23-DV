cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# ament_auto_find_build_dependencies()

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

rosidl_generate_interfaces(${PROJECT_NAME}
  ## Can-USB sent msgs
  "msg/CanInterface/TxSystemState.msg"
  "msg/CanInterface/TxControlCommand.msg"
  "msg/CanInterface/TxVehicleVariables.msg"   # Not actually used
  "msg/CanInterface/TxSteeringParams.msg"   # Not actually used

  ## Can-USB receive msgs
  "msg/CanInterface/RxWheelSpeed.msg"
  "msg/CanInterface/RxVehicleSensors.msg"
  "msg/CanInterface/RxSteeringAngle.msg"

  ## Master node msgs
  "msg/NodeSync.msg"

  ## Velocity Estimation msgs
  "msg/VelEstimation.msg"

  ## Perception msgs
  "msg/AcquisitionMessage.msg"
  "msg/Perception2Slam.msg"
  "msg/AutoExposureMessage.msg"

  ## P23 Status Messages
  "msg/MissionSelection.msg"
  "msg/AutonomousStatus.msg"
  "msg/DriverlessStatus.msg"
  "msg/Mission.msg"
  "msg/LifecycleNodeStatus.msg"
  "msg/LifecycleNodeTransitionState.msg"
  "msg/DriverlessTransition.msg"

  # Path Planning Messages
  "msg/ConeStruct.msg"
  "msg/LocalMapMsg.msg"
  "msg/Point2Struct.msg"
  "msg/PoseMsg.msg"
  "msg/WaypointsMsg.msg"

  # Services
  "srv/GetFrequencies.srv"
  "srv/DriverlessStatus.srv"
  "srv/InsMode.srv"
  "srv/DriverlessTransition.srv"
  "srv/ResurrectNode.srv"
  "srv/ResurrectOrder.srv"
  "srv/SetTotalLaps.srv"

  ## Messages for P22 testing
  "msg/P22/WheelSpeed.msg"
  "msg/P22/SteeringAngle.msg"
  "msg/P22/BrakePressure.msg"

  # Actions
  "action/DriverlessTransition.action"

  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
