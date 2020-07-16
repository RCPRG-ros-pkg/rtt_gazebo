cmake_minimum_required(VERSION 2.8.6)

project(simulation_control_msgs)

find_package(catkin REQUIRED
  COMPONENTS std_msgs message_generation actionlib_msgs)

add_action_files(
  DIRECTORY action
  FILES
  SimControl.action
)

#add_message_files(FILES )
add_service_files(
  FILES
  EnableSim.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
  )


catkin_package(
  CATKIN_DEPENDS std_msgs message_runtime)
