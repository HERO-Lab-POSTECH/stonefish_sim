cmake_minimum_required(VERSION 3.0.2)
project(stonefish_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  BeaconInfo.msg
  DVL.msg
  DVLBeam.msg
  Event.msg
  EventArray.msg
  INS.msg
  Int32Stamped.msg
  NEDPose.msg
  ThrusterState.msg
)

add_service_files(
  FILES
  Respawn.srv
  SonarSettings.srv
  SonarSettings2.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs
)
