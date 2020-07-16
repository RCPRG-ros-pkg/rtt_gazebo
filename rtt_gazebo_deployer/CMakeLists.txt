cmake_minimum_required(VERSION 2.8.3)
project(rtt_gazebo_deployer)

## Find catkin macros and  braries
find_package(catkin REQUIRED COMPONENTS 
  rtt_ros
  std_srvs
  simulation_control_msgs
  cmake_modules subsystem_deployer subsystem_common)

find_package(OROCOS-RTT REQUIRED COMPONENTS rtt-scripting rtt-transport-corba)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS thread REQUIRED)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${catkin_LIBRARY_DIRS}
  )

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIR}
  ${USE_OROCOS_INCLUDE_DIRS}
  )

orocos_use_package( ocl-taskbrowser )
orocos_use_package( ocl-deployment )
orocos_use_package( ocl-logging )
orocos_use_package( orocos-rtt-corba )
orocos_use_package( rtt-typekit )

## Gazebo RTT plugin
add_definitions(-DRTT_COMPONENT)
add_library(${PROJECT_NAME} 
  src/gazebo_deployer_model_plugin.cpp)

target_link_libraries(${PROJECT_NAME}
  ${GAZEBO_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OROCOS-RTT_LIBRARIES} 
  ${OROCOS-RTT_RTT-SCRIPTING_LIBRARY} 
  ${OROCOS-RTT_RTT-TRANSPORT-CORBA_LIBRARY}
  ${USE_OROCOS_LIBRARIES}
  ${Boost_LIBRARIES}
)

#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

orocos_generate_package(
#  CATKIN_DEPENDS rtt_ros subsystem_deployer
#  CATKIN_DEPENDS subsystem_deployer
)

install(TARGETS
  ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )

