project(PTAML CXX C)
cmake_minimum_required(VERSION 2.8)

# Set default built type
set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING
    "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel.")
    
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
    add_definitions(-pthread -O3 -Wall -march=core2 -mtune=core2 -msse3)
	#add_definitions(-D_LINUX -D_REENTRANT -O3 -Wall -march=core2 -mtune=core2 -msse3 -fno-strict-aliasing) # Old
    
    #set(CMAKE_EXE_LINKER_FLAGS -pg) 
endif()

# Gstreamer
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-0.10)

# AR Toolkit
set(ARTOOLKIT_DIR "/home/daniel/Code/ARToolKit/" CACHE STRING
    "Set the installation directory of AR Toolkit.")

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
	glut
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
	Utils.cc
	ARDriver.cc
	ARToolkit.cc
	ATANCamera.cc
	Bundle.cc
	GLWindow2.cc
	GLWindowMenu.cc
	HomographyInit.cc
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
	PositionHold.cc
	Relocaliser.cc
	ShiTomasi.cc
	SmallBlurryImage.cc
	System.cc
	tinyxml.cc
	tinyxmlerror.cc
	tinyxmlparser.cc
	Tracker.cc
	MKProtocol.cc
	MKConnection.cc
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
# Data Files
#

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/settings.cfg DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Data DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
