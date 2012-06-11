project(PTAML CXX C)
cmake_minimum_required(VERSION 2.8)

# Set default built type
set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel.")
    
# Target platfrom
set(TARGET_PLATFORM "HexaMac" CACHE STRING
    "Set the target platform (HexaMac/Other).")
    
    
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
    if(TARGET_PLATFORM STREQUAL "HexaMac")
    	message(STATUS "Building with a performance profile optimized for the HexaMac")
    	add_definitions(-g -pthread -O6 -Wall -march=corei7 -mtune=corei7 -msse4.2 -mfpmath=sse)
    else()
    	message(STATUS "Building with a general peformance profile: ${TARGET_PLATFORM}")
    	add_definitions(-g -pthread -O3 -Wall -march=core2 -mtune=core2 -msse3)
    endif()
    
    #set(CMAKE_EXE_LINKER_FLAGS -fopenmp) 
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

# AR Toolkit   
find_path(ARTOOLKIT_DIR 
	NAMES include/AR 
	HINTS ~/ARToolKit ~/Code/ARToolKit
	DOC "Set the installation directory of AR Toolkit.")
	
if(NOT ARTOOLKIT_DIR)
	message(FATAL_ERROR "Could not find ARToolKit, please set 'ARTOOLKIT_DIR' manually")
endif()

# Include paths
include_directories(
    "${ARTOOLKIT_DIR}/include"
    ${GSTREAMER_INCLUDE_DIRS}
)

link_directories(
	"${ARTOOLKIT_DIR}/lib"
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
)

set(VIDEO_SOURCE_SRC
	VideoSource_Linux_Gstreamer_File.cc
	VideoSource_Linux_V4L.cc
)

#
# PTAML
#

set(PTAML_SRC
	ARDriver.cc
	ARToolkit.cc
	ATANCamera.cc
	Bundle.cc
	FeatureGrid.cc
	Frontend.cc
	GLWindow2.cc
	GLWindowMenu.cc
	HomographyInit.cc
	InitialTracker.cc
	KeyFrame.cc
	main.cc
	Map.cc
	MapLockManager.cc
	MapMaker.cc
	MapPoint.cc
	MapSerializer.cc
	MapViewer.cc
	MD5.cc
	MD5Wrapper.cc
	MiniPatch.cc
	PatchFinder.cc
	TargetController.cc
	Rectangle.cc
	Relocaliser.cc
	ScaleMarkerTracker.cc
	ShiTomasi.cc
	SmallBlurryImage.cc
	System.cc
	Timing.cc
	tinyxml.cc
	tinyxmlerror.cc
	tinyxmlparser.cc
	Tracker.cc
	Utils.cc
	MKProtocol.cc
	MKConnection.cc
	MikroKopter.cc
	rs232.c
	VideoSource.cc
	${VIDEO_SOURCE_SRC}
)

add_executable(PTAML ${PTAML_SRC})
target_link_libraries(PTAML ${LIBS})

#
# CameraCalibrator
#

set(CC_SRC
	GLWindow2.cc
	VideoSource.cc
	GLWindowMenu.cc
	CalibImage.cc
	CalibCornerPatch.cc
	ATANCamera.cc
	CameraCalibrator.cc
	${VIDEO_SOURCE_SRC}
)

add_executable(CameraCalibrator ${CC_SRC})
target_link_libraries(CameraCalibrator ${LIBS})

#
# Other programs
#

set(FASTTEST_SRC
	Timing.cc
	FastTest.cc
)

add_executable(FastTest ${FASTTEST_SRC})
target_link_libraries(FastTest ${LIBS})

#
# Data Files
#

if(NOT EXISTS ${CMAKE_CURRENT_BINARY_DIR}/settings.cfg)
	file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/settings.cfg DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
endif()

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
