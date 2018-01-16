cmake_minimum_required(VERSION 2.8.3)
project(iiwa)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
	std_msgs
	sensor_msgs
	geometry_msgs 
	trajectory_msgs 
	roscpp rospy 
	actionlib actionlib_msgs 
	control_msgs
	arm_manager
	message_generation
  iiwa_ros
	)


find_package(Eigen3 REQUIRED)

set(eigen_functionalities_DIR "/usr/local/lib/cmake/eigen_functionalities")
find_package(eigen_functionalities REQUIRED)



## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetContainerPose.srv
  GetPiecePose.srv
  GraspInfo.srv
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES BinPicking.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   actionlib_msgs
   geometry_msgs
 )







catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lib_state_machine_iiwa
 CATKIN_DEPENDS message_runtime actionlib_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${EIGEN_FUNCTIONALITIES_INCLUDE_DIRS} 
)



add_library(lib_state_machine_iiwa src/state_machine_iiwa_helper.cpp)
add_library(lib_binpicking_iiwa src/binPickingAction_class.cpp)



add_executable(camera_server src/getContainerPose.cpp)

add_executable(state_machine_iiwa src/state_machine_iiwa.cpp)

add_executable(client_control src/client_control.cpp)
add_executable(manager_example src/manager_example.cpp)



target_link_libraries(lib_state_machine_iiwa ${catkin_LIBRARIES})

target_link_libraries(lib_binpicking_iiwa lib_state_machine_iiwa ${catkin_LIBRARIES})


target_link_libraries(state_machine_iiwa lib_binpicking_iiwa ${catkin_LIBRARIES})

target_link_libraries(camera_server ${catkin_LIBRARIES})

target_link_libraries(client_control ${catkin_LIBRARIES})

target_link_libraries(manager_example ${catkin_LIBRARIES})





add_dependencies(lib_state_machine_iiwa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(lib_binpicking_iiwa lib_state_machine_iiwa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



add_dependencies(state_machine_iiwa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} lib_binpicking_iiwa iiwa_generate_messages_cpp)

add_dependencies(client_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} lib_state_machine_iiwa iiwa_generate_messages_cpp)

add_dependencies(manager_example ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} lib_state_machine_iiwa iiwa_generate_messages_cpp)


add_dependencies(camera_server iiwa_gencpp)


add_executable(setting_control_mode_sim src/setting_control_mode.cpp)
add_dependencies(setting_control_mode_sim iiwa_msgs_generate_messages_cpp)
target_link_libraries(setting_control_mode_sim ${catkin_LIBRARIES})

add_executable(command_robot_sim src/command_robot_sim.cpp)
add_dependencies(command_robot_sim iiwa_msgs_generate_messages_cpp)
target_link_libraries(command_robot_sim ${catkin_LIBRARIES})

