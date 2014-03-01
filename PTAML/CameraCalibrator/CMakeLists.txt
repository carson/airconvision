# Libraries
set(CC_LIBS
	${GSTREAMER_LIBRARIES}
	gstapp-0.10
	GVars3
	cvd
	GLU 
	GL
	lapack
	blas
)

include_directories(
    ../src
)

#
# CameraCalibrator
#

set(CC_SRC
	../src/GLWindow2.cc
	../src/VideoSource.cc
	../src/GLWindowMenu.cc
	../src/ATANCamera.cc
	../src/Timing.cc
	CalibImage.cc
	CalibCornerPatch.cc
	CameraCalibrator.cc
	${VIDEO_SOURCE_SRC}
)

add_executable(CameraCalibrator ${CC_SRC})
target_link_libraries(CameraCalibrator ${CC_LIBS})
target_link_libraries(CameraCalibrator flycapture)

