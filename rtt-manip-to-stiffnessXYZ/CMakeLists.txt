cmake_minimum_required(VERSION 2.8.3)
project(rtt-manip-to-stiffnessXYZ)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
# set (CMAKE_CXX_STANDARD 11)

include_directories(${PROJECT_SOURCE_DIR}/include)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

# Example find_package because I'm lazy
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

# orocos_install_headers(includes)

orocos_component(${CMAKE_PROJECT_NAME} 
				 ${PROJECT_SOURCE_DIR}/src/ManipToStiffness.cpp
				 ${PROJECT_SOURCE_DIR}/include/ManipToStiffness.hpp
				)

target_link_libraries(${CMAKE_PROJECT_NAME}
						${RST-RT_LIBRARIES}
                        ${orocos_kdl_LIBRARIES}
					 )

orocos_generate_package()
