cmake_minimum_required(VERSION 2.8.3)
project(rtt_simulation_control_msgs)

find_package(catkin REQUIRED COMPONENTS rtt_roscomm simulation_control_msgs)

ros_generate_rtt_typekit(simulation_control_msgs)
#ros_generate_rtt_service_proxies(simulation_control_msgs)

orocos_generate_package(
  DEPENDS simulation_control_msgs
  DEPENDS_TARGETS rtt_roscomm rtt_std_msgs rtt_actionlib_msgs

)
