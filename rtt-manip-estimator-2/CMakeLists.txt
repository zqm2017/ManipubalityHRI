cmake_minimum_required(VERSION 2.8.3)
project(rtt-manip-estimator-2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# set (CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH} /opt/ros/kinetic")
#find_package(kdl_conversions REQUIRED)
find_package(geometry_msgs REQUIRED)
IF (geometry_msgs_FOUND)
        message(STATUS "geometry_msgs found. Version: ${geometry_msgs_VERSION}")
        include_directories(BEFORE SYSTEM ${geometry_msgs_INCLUDE_DIRS})
        add_definitions(${geometry_msgs_CFLAGS})
        link_directories(${geometry_msgs_LIBRARY_DIR})
ENDIF()

INCLUDE_DIRECTORIES(/home/sgo/ws/underlay/src/rtt_ros_integration/rtt_rosclock/include/)
INCLUDE_DIRECTORIES(/home/sgo/ws/underlay/src/rtt_ros_integration/rtt_roscomm/include/)
link_directories(/home/sgo/ws/underlay/devel/lib)

#include_directories(${geometry_msgs_INCLUDE_DIRS})

find_package(RST-RT REQUIRED)
IF (RST-RT_FOUND)
	message(STATUS "RST-RT found. Version: ${RST-RT_VERSION}")
	include_directories(BEFORE SYSTEM ${RST-RT_INCLUDE_DIRS})
	add_definitions(${RST-RT_CFLAGS})
	link_directories(${RST-RT_LIBRARY_DIRS})
ENDIF()


find_package(orocos_kdl REQUIRED)
IF (orocos_kdl_FOUND)
    message(STATUS "orocos_kdl found. Version: ${orocos_kdl_VERSION}")
    include_directories(BEFORE SYSTEM ${orocos_kdl_INCLUDE_DIRS})
    add_definitions(${orocos_kdl_CFLAGS})
    link_directories(${orocos_kdl_LIBRARY_DIRS})
ENDIF()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)

find_package(urdfdom REQUIRED)
IF (urdfdom_FOUND)
        message(STATUS "urdfdom found. Version: ${Boost_VERSION}")
        include_directories(BEFORE SYSTEM ${urdfdom_INCLUDE_DIRS})
        add_definitions(${urdfdom_CFLAGS})
        link_directories(${urdfdom_LIBRARY_DIR})
ENDIF()


# orocos_install_headers(includes)

add_subdirectory(urdf-to-kdl)
include_directories(${KDL_Parser_INCLUDE_DIRS})


orocos_component(${CMAKE_PROJECT_NAME} 
				 ${PROJECT_SOURCE_DIR}/src/ManipEstimator.cpp
				 ${PROJECT_SOURCE_DIR}/include/ManipEstimator.hpp
				)
#target_link_libraries(${CMAKE_PROJECT_NAME} KDL_Parser)
target_link_libraries(${CMAKE_PROJECT_NAME}
                        ${RST-RT_LIBRARIES}
                        ${orocos_kdl_LIBRARIES}
                        ${urdfdom_LIBRARIES}
                        KDL_Parser
                        ${kdl_conversions_LIBRARIES}
                        librtt_rostopic-gnulinux.so
                        librtt_rosclock-gnulinux.so
					 )

orocos_generate_package()
