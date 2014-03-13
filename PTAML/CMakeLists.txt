project(PTAML CXX C)
cmake_minimum_required(VERSION 2.8)
# Set default built type
set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel.")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
    
# Target platfrom
option(FULL_OPTIMIZATION "Enable all optimization flags" OFF)
    
# The CDT Error Parser cannot handle error messages that span more than one line, which is the default gcc behavior. 
# This forces gcc to generate single line error messages with no line wrapping.  
IF(CMAKE_COMPILER_IS_GNUCC)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCC)
IF(CMAKE_COMPILER_IS_GNUCXX)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fmessage-length=0")
ENDIF(CMAKE_COMPILER_IS_GNUCXX)

# This enables Eclipse to discover the include dirs automatically
SET(CMAKE_VERBOSE_MAKEFILE ON)

# Settings
if(CMAKE_COMPILER_IS_GNUCXX)
    # Add support for C++11/0x
    add_definitions(-std=gnu++0x)
    
    # Optimizations etc
    if(FULL_OPTIMIZATION)
    	message(STATUS "Building with a performance profile optimized for the HexaMac")
    	add_definitions(-g -pthread -O6 -Wall -march=corei7 -mtune=corei7 -msse -msse2 -msse3 -msse4 -msse4.2)
    else()
    	message(STATUS "Building with a general performance profile.")
    	add_definitions(-g -pthread -O3 -Wall -march=native -mtune=native)
    endif()
    
    #set(CMAKE_EXE_LINKER_FLAGS -fopenmp) 
endif()

if(UNIX)
  add_definitions(-D_LINUX)
endif()

find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Gstreamer
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)

# Include paths
include_directories(
    ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(
	${GSTREAMER_LIBRARY_DIRS}
)

# Libraries
set(LIBS
	${GSTREAMER_LIBRARIES}
	gstapp-0.10
	GVars3
	cvd
	ARgsub
	AR 
	GLU 
	GL
	lapack
	blas
    flycapture
)

set(VIDEO_SOURCE_SRC
	${CMAKE_CURRENT_SOURCE_DIR}/src/VideoSource_Linux_Gstreamer_File.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/VideoSource_Linux_V4L.cc
	${CMAKE_CURRENT_SOURCE_DIR}/src/VideoSource_Image.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/src/VideoSource_FlyCapture.cc
)

#
# PTAML
#

add_subdirectory(src)


#
# CameraCalibrator
#

add_subdirectory(CameraCalibrator)

#
# Other programs
#

add_subdirectory(FeatureScatterPlot)

#
# Data Files
#

if(NOT EXISTS ${CMAKE_SOURCE_DIR}/settings.cfg)
	#file(COPY ${CMAKE_SOURCE_DIR}/settings_template.cfg DESTINATION ${CMAKE_SOURCE_DIR}/settings.cfg)
	configure_file(${CMAKE_SOURCE_DIR}/settings_template.cfg ${CMAKE_SOURCE_DIR}/settings.cfg COPYONLY)
endif()

