cmake_minimum_required(VERSION 2.8.3)
project(bta_ros)

## check required modules
SET(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")
FIND_PACKAGE(bta REQUIRED)

#find_package(Boost REQUIRED COMPONENTS system thread)

#FIND_PACKAGE(OpenCV REQUIRED)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  sensor_msgs
  std_msgs
  pcl_ros
  pcl_conversions
  image_transport
  camera_info_manager
  nodelet
)

generate_dynamic_reconfigure_options(
  cfg/bta_ros.cfg
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES bta_ros
 CATKIN_DEPENDS dynamic_reconfigure roscpp sensor_msgs std_msgs pcl_ros pcl_conversions image_transport camera_info_manager nodelet
  DEPENDS bta
)

###########
## Build ##
###########

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${bta_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(bta_ros
src/bta_ros.cpp
)
target_link_libraries(bta_ros bta_ros ${bta_LIBRARIES} ${catkin_LIBRARIES} )
add_dependencies(bta_ros ${PROJECT_NAME}_gencfg) 

add_library(BtaRosNodelet
src/bta_ros_nodelet.cpp
)
target_link_libraries(BtaRosNodelet bta_ros ${catkin_LIBRARIES} ${Boost_LIBRARIES} )

## Declare a cpp executable
add_executable(bta_ros_node src/bta_ros_node.cpp)

target_link_libraries(bta_ros_node bta_ros ${catkin_LIBRARIES} )
#set_target_properties(bta_ros_node PROPERTIES OUTPUT_NAME bta_ros)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
 install(TARGETS bta_ros bta_ros_node BtaRosNodelet
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
	install(FILES nodelet_plugins.xml
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sentis_m100.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
