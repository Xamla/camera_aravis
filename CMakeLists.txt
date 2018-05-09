# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)

# Enable C++11
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra")
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

project(camera_aravis)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport camera_info_manager genmsg)
find_package(Aravis REQUIRED)
find_package(GLIB2 REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
  Capture.srv
  GetConnectedDevice.srv
  SendCommand.srv
)

generate_messages(
   DEPENDENCIES
   sensor_msgs
)

catkin_package(
    DEPENDS Aravis GLIB2
    CATKIN_DEPENDS roscpp sensor_msgs image_transport camera_info_manager
    INCLUDE_DIRS include
    LIBRARIES
)

include_directories(include ${catkin_INCLUDE_DIRS} ${ARAVIS_INCLUDE_PATH} ${GLIB2_INCLUDE_DIRS})

message(${ARAVIS_LIBRARY})

add_executable(${PROJECT_NAME}_node src/camnode.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${ARAVIS_LIBRARY} glib-2.0 gmodule-2.0 gobject-2.0)
add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_gencfg)
