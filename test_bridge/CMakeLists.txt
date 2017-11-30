
cmake_minimum_required(VERSION 2.8.3)

project(test_bridge)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

include_directories(${PROJECT_SOURCE_DIR}/include)


find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} /opt/ros/kinetic")

include_directories(/opt/ros/kinetic/include/orocos/)

LINK_DIRECTORIES(/opt/ros/kinetic/lib/)

message(STATUS "------------------ ${include_directories}")
find_package(geometry_msgs REQUIRED)
IF (geometry_msgs_FOUND)
        message(STATUS "geometry_msgs found. Version: ${geometry_msgs_VERSION}")
        include_directories(BEFORE SYSTEM ${geometry_msgs_INCLUDE_DIRS})
        add_definitions(${geometry_msgs_CFLAGS})
        link_directories(${geometry_msgs_LIBRARY_DIR})
ENDIF()



find_package(kdl_conversions)
IF (kdl_conversions_FOUND)
    message(STATUS "kdl_conversions found. Version: ${kdl_conversions_VERSION}")
    INCLUDE_DIRECTORIES(${kdl_conversions_INCLUDE_DIRS})
    add_definitions(${kdl_conversions_CFLAGS})
    link_directories(${kdl_conversions_LIBRARY_DIRS})
ENDIF()


find_package(orocos_kdl REQUIRED)
IF (orocos_kdl_FOUND)
    message(STATUS "orocos_kdl found. Version: ${orocos_kdl_VERSION}")
    INCLUDE_DIRECTORIES(${orocos_kdl_INCLUDE_DIRS})
ENDIF()

include( ${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )



orocos_component(test_bridge src/test-component.hpp src/test-component.cpp)
orocos_install_headers(test-component.hpp) 
# Generates and installs our package. Must be the last statement such
# that it can pick up all above settings.
#
target_link_libraries(${CMAKE_PROJECT_NAME}
                      librtt_rostopic-gnulinux.so
                      librtt_rosclock-gnulinux.so)
orocos_generate_package()

