cmake_minimum_required(VERSION 3.18)
set(PROJECT_NAME VINS-Mobile)
project(${PROJECT_NAME})

set(CMAKE_CXX_STANDARD 17)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)
set(THIRD_PARTY_PATH "${CMAKE_SOURCE_DIR}/thirdparty")

include(SetPlatformVarsInternal)
include(FindJsonInternal)
include(FindBoostInternal)
include(FindCeresInternal)
include(FindEigen3Internal)
include(FindOpenCVInternal)
include(FindRabbitMQInternal)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/ios_player)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty)
include_directories(${CMAKE_SOURCE_DIR}/estimator)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/DBoW)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/DUtils)
include_directories(${CMAKE_SOURCE_DIR}/thirdparty/DVision)
include_directories(${CMAKE_SOURCE_DIR}/estimator/factor)
include_directories(${CMAKE_SOURCE_DIR}/estimator/feature)
include_directories(${CMAKE_SOURCE_DIR}/estimator/initial)
include_directories(${CMAKE_SOURCE_DIR}/estimator/utility)
include_directories(${CMAKE_SOURCE_DIR}/estimator/loop_closure)
include_directories(${CMAKE_SOURCE_DIR}/estimator/visualization)

link_directories(${CMAKE_BUILD_DIR}/lib)

message(STATUS "EIGEN3_INCLUDE_DIR: ${EIGEN3_INCLUDE_DIR}")

add_subdirectory(thirdparty)

if(TARGET DBoW)
	add_subdirectory(estimator)
endif()

if(TARGET DBoW AND TARGET vins_estimator)
	add_subdirectory(ios_player)
endif()
