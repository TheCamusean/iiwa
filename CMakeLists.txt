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
	)


find_package(Eigen3 REQUIRED)

set(eigen_functionalities_DIR "/usr/local/lib/cmake/eigen_functionalities")
find_package(eigen_functionalities REQUIRED)



## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetContainerPose.srv
  GetPiecePose.srv
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


add_executable(camera_server src/getContainerPose.cpp)

add_executable(state_machine_iiwa src/state_machine_iiwa.cpp)

add_executable(client_control src/client_control.cpp)


target_link_libraries(lib_state_machine_iiwa ${catkin_LIBRARIES})

target_link_libraries(state_machine_iiwa lib_state_machine_iiwa ${catkin_LIBRARIES})

target_link_libraries(camera_server ${catkin_LIBRARIES})

target_link_libraries(client_control ${catkin_LIBRARIES})




add_dependencies(lib_state_machine_iiwa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_dependencies(state_machine_iiwa ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} lib_state_machine_iiwa iiwa_generate_messages_cpp)

add_dependencies(client_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} lib_state_machine_iiwa iiwa_generate_messages_cpp)


add_dependencies(camera_server iiwa_gencpp)
