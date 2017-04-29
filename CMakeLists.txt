cmake_minimum_required(VERSION 3.5.1)

project(Roboty_autonomiczne_Nowoczesne_sensory)

SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

set(CMAKE_CXX_STANDARD 11)
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
SET(CMAKE_BUILD_TYPE Debug)

# Cmake
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

# PCL Library
find_package(PCL 1.8 REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV Library
FIND_PACKAGE(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})

# Eigen3 Library
FIND_PACKAGE(Eigen3 REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})

################ MODELS ################

file(GLOB MODEL_SOURCES
        "${CMAKE_SOURCE_DIR}/src/models/*.cpp"
        )
file(GLOB MODEL_HEADERS
        "${CMAKE_SOURCE_DIR}/include/models/*.h"
        )
ADD_LIBRARY(Models STATIC ${MODEL_SOURCES} ${MODEL_HEADERS})
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/models/)

#########################################

################ UTILS ################

file(GLOB UTILS_SOURCES
        "${CMAKE_SOURCE_DIR}/src/utils/*.cpp"
        )
file(GLOB UTILS_HEADERS
        "${CMAKE_SOURCE_DIR}/include/utils/*.h"
        )
ADD_LIBRARY(Utils STATIC ${UTILS_SOURCES} ${UTILS_HEADERS} include/utils/edited_lccp_segmentation.h src/utils/edited_lccp_segmentation.cpp)
INSTALL(FILES ${DEFS_HEADERS} DESTINATION include/utils/)

#########################################

################# Program ###############

file(GLOB PROGRAM_SOURCES
        "${CMAKE_SOURCE_DIR}/src/program/*.cpp"
        )
file(GLOB PROGRAM_HEADERS
        "${CMAKE_SOURCE_DIR}/include/program/*.h"
        )
ADD_EXECUTABLE(Program ${PROGRAM_SOURCES} ${PROGRAM_HEADERS})
TARGET_LINK_LIBRARIES(Program Models Utils ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

#########################################


