###############################################################################
# Declare the project
###############################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(djiosdk-base-station)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -g -O0")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lboost_system")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lboost_program_options")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -luhd")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lboost_thread")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lmraa")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)


###############################################################################
# Create DJI Onboard-SDK targets
###############################################################################

if(NOT ONBOARDSDK_SOURCE)
  set(ONBOARDSDK_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/lib/Onboard-SDK")
endif()

if(NOT DISTRO_VERSION)
    set(DISTRO_VERSION UNKNOWN)
endif()

add_subdirectory("${ONBOARDSDK_SOURCE}/osdk-core")
add_subdirectory("${ONBOARDSDK_SOURCE}/osdk-wrapper")
add_subdirectory("${ONBOARDSDK_SOURCE}/platform/linux")

target_include_directories(djiosdk-core PUBLIC "${ONBOARDSDK_SOURCE}/osdk-core/inc")
target_include_directories(djiosdk-wrapper PUBLIC "${ONBOARDSDK_SOURCE}/osdk-wrapper/inc")
target_include_directories(djiosdk-platform PUBLIC "${ONBOARDSDK_SOURCE}/platform/linux/inc")


###############################################################################
# Create project main target
###############################################################################

add_subdirectory(baseStation)
