# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(camera_aravis)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(catkin REQUIRED COMPONENTS
             roscpp sensor_msgs std_msgs
             image_transport camera_info_manager
             message_generation cv_bridge xamla_sysmon_msgs)

find_package(Aravis REQUIRED)
find_package(GLIB2 REQUIRED)

add_service_files(
  DIRECTORY
    srv
  FILES
    Capture.srv
    GetConnectedDevices.srv
    SendCommand.srv
    SetIO.srv
)

generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
)

catkin_package(
    DEPENDS Aravis GLIB2
    CATKIN_DEPENDS xamla_sysmon_msgs roscpp message_runtime std_msgs sensor_msgs image_transport camera_info_manager cv_bridge
    INCLUDE_DIRS include
    LIBRARIES
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Aravis_INCLUDE_DIRS} ${GLIB2_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_node
    src/cameranode.cpp src/genicamfeatures.cpp
    src/cameramanager.cpp src/genicam.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${Aravis_LIBRARIES} glib-2.0 gmodule-2.0 gobject-2.0)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/service_call_examples/capture_service_client
  scripts/service_call_examples/collect_image_and_joint_values
  scripts/service_call_examples/getconnecteddevices_service_client
  scripts/service_call_examples/sendCommand_service_client
  scripts/service_call_examples/setIO_service_client
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)
