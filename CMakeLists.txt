cmake_minimum_required(VERSION 2.8.3)
project(conveyor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mrobot_msgs
  roscan
)

###########
## Build ##
###########
set (CMAKE_CXX_FLAGS "-std=c++11")

include_directories(
  ${catkin_INCLUDE_DIRS}
  include/conveyor
)

add_executable( conveyor_node
    src/main.cpp
    src/conveyor.cpp
)

catkin_package(
    CATKIN_DEPENDS roscpp std_msgs
)

target_link_libraries(conveyor_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS conveyor_node
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
    USE_SOURCE_PERMISSIONS
    )

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

