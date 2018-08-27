cmake_minimum_required(VERSION 2.8.3)
project(noah_sensors)

find_package(catkin REQUIRED COMPONENTS
    std_msgs
    roscpp
    message_generation
    mrobot_driver_msgs
    #range_sensor_layer
    sonar_msgs
    roscan
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

add_message_files(
  FILES
  sonar_msgs.msg
)


###########
## Build ##
###########
set (CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/noah_sensors
)

add_executable( noah_sensors
    src/main.cpp
    src/ultrasonic.cpp
    src/micro_laser.cpp
    src/hall.cpp
    src/common.cpp
)

add_dependencies(noah_sensors
    ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(noah_sensors
  ${catkin_LIBRARIES} 
)

#############
## Install ##
#############
install(TARGETS noah_sensors
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        USE_SOURCE_PERMISSIONS)

#############
## Testing ##
#############

