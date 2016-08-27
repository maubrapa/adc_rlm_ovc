#=============================================================================
# Copyright 2014 de Paula, Mauricio Braga.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#
# Last updated 25/08/2014
#=============================================================================

project( adc_rlm_ovc)

cmake_minimum_required(VERSION 2.8)

### Find required libs ###
find_package( OpenCV REQUIRED )

find_package(OpenMP)
if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

add_executable( adc_rlm_ovc
    src/Tcc.h
    src/Tcc.cpp
    src/ConfigParameters.h
    src/FileStorageComplements.h
    src/GaussianClassifier.cpp
    src/GaussianClassifier.h
    src/Util.cpp
    src/Util.h
    src/vcvice.cpp
    src/vcvice.h
    src/VideoProcessor.cpp
    src/VideoProcessor.h
    src/linearParabolicModel/_aux.h
    src/linearParabolicModel/CLane.cpp
    src/linearParabolicModel/CLane.h
    src/linearParabolicModel/CMatrix.h
    src/linearParabolicModel/CPeak.h
    src/linearParabolicModel/LineSegment2D.h
    src/main.cpp
    src/RoadMark.cpp
    src/RoadMark.h
    src/Definitions.h
    src/conf/config.xml
    src/conf/i5s.xml
)

include_directories("${CMAKE_SOURCE_DIR}/src/"
                    "${CMAKE_SOURCE_DIR}/src/linearParabolicModel/")

### Set some options for cmake ###
set(GCC_FLAGS "-std=c++0x -O3")

### Tell cmake that the binaries goes in the binary directory ###
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_BINARY_DIR}/bin)

add_definitions(${GCC_FLAGS})
target_link_libraries( adc_rlm_ovc ${OpenCV_LIBS} )
