cmake_minimum_required(VERSION 2.8.3)
project(table_broadcaster)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED tf std_msgs optitrack_rviz message_generation)

add_service_files(
  FILES
  Transform_cmd.srv
  String_cmd.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

set(CMAKE_CXX_FLAGS "-O2 -O3 -std=c++11 -mtune=native -fPIC -Wall")        ## Optimize
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")

set(INCLUDE_V2R /home/guillaume/roscode/catkin_ws/src/vision_to_rviz)


set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(HEADER_FILES ${INCLUDE_DIR}/table_broadcaster/table_broadcaster.h)

set(SRC_FILES    ${SRC_DIR}/table_broadcaster.cpp
                 ${SRC_DIR}/run.cpp    )


message(STATUS "current_source dir = ${CMAKE_CURRENT_SOURCE_DIR}")
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include ${INCLUDE_V2R}/include/)
include_directories(${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME} ${SRC_FILES} ${HEADER_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})


