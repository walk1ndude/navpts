cmake_minimum_required(VERSION 2.8.3)
project(navpts_group)

set(CMAKE_BUILD_TYPE, Release)

find_package(catkin REQUIRED COMPONENTS roscpp cv_bridge std_msgs image_transport ardrone_autonomy message_generation geometry_msgs)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui calib3d)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS message_runtime
)

add_message_files(FILES PoseID.msg PoseArrayID.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(INCLUDE_DIRS include)

set(SOURCES src/navpts.cpp
    src/utils.cpp
    src/drone.cpp
    src/EKFDrone.cpp
    src/APDrone.cpp
    src/ardrone.cpp

    include/drone.h
    include/EKFDrone.h
    include/APDrone.h
    include/ardrone.h
    include/utils.h
    include/PVFilter.h
)
## Declare a cpp executable
add_executable(navpts_group ${SOURCES})
add_dependencies(navpts_group ${PROJECT_NAME}_gencpp)
#add_definitions(-std=c++11)

target_link_libraries(navpts_group ${catkin_LIBRARIES} ${OpenCV_LIBS} -laruco)
