cmake_minimum_required(VERSION 3.8)
project(robot_arm_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RobotStatus.msg"
  "msg/PlcStatus.msg"
  "srv/ServoOn.srv"
  "srv/WaitDuration.srv"
  "srv/Jog.srv"
  "srv/Home.srv"
  "srv/Teaching.srv"
  "srv/EStop.srv"
  "srv/Recover.srv"
  "srv/PlcSetOutput.srv"
  "action/MoveJ.action"
  "action/MoveL.action"
  DEPENDENCIES std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
