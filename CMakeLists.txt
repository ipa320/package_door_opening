cmake_minimum_required(VERSION 3.0.2)
project(pitasc_pivot_taubin)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
    cppitasc
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
    LIBRARIES ${PROJECT_NAME}_monitors ${PROJECT_NAME}_scripts 
    CATKIN_DEPENDS cppitasc
    DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}_monitors
    src/monitors/monitor_relative_angular_distance.cpp
)
target_link_libraries(${PROJECT_NAME}_monitors
    ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_scripts
    src/scripts/script_temp_frame_offset.cpp
    src/scripts/script_temp_frame_from_taubin.cpp
)
target_link_libraries(${PROJECT_NAME}_scripts
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_monitors ${PROJECT_NAME}_scripts
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY skills
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY examples
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
