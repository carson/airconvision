# AR Toolkit   
find_path(ARTOOLKIT_DIR 
	NAMES include/AR 
	HINTS /opt/ARToolKit ~/ARToolKit ${CMAKE_SOURCE_DIR}/EXTERNAL/ARToolKit
	DOC "Set the installation directory of AR Toolkit.")
	
if(NOT ARTOOLKIT_DIR)
	message(FATAL_ERROR "Could not find ARToolKit, please set 'ARTOOLKIT_DIR' manually")
endif()

# Include paths
include_directories(
    "${ARTOOLKIT_DIR}/include"
	"/usr/local/flycapture"
)

link_directories(
	"${ARTOOLKIT_DIR}/lib"
	"/usr/lib"
)

# Libraries
set(PTAML_LIBS
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
	opencv_core
	opencv_imgproc
	opencv_highgui
	opencv_ml
	opencv_video
	opencv_features2d
	opencv_calib3d
	opencv_objdetect
	opencv_contrib
	opencv_legacy
	opencv_flann
)

#
# PTAML
#

set(AGAST_SRC
    agast/nonMaximumSuppression.cc
    agast/agast5_8_nms.cc
    agast/agast5_8.cc
    agast/agast7_12d_nms.cc
    agast/agast7_12d.cc
    agast/agast7_12s_nms.cc
    agast/agast7_12s.cc
    agast/oast9_16_nms.cc
    agast/oast9_16.cc
)

set(PTAML_SRC
	${AGAST_SRC}
	ARToolkit.cc
	ATANCamera.cc
	Bundle.cc
	FeatureGrid.cc
	FrameGrabber.cc
	Frontend.cc
	FrontendRenderer.cc
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
	PerformanceMonitor.cc
	TargetController.cc
	Rectangle.cc
	Relocaliser.cc
	ScaleMarkerTracker.cc
	ShiTomasi.cc
	SmallBlurryImage.cc
	StereoPlaneFinder.cc
	System.cc
	Swarm.cc
	Timing.cc
	tinyxml.cc
	tinyxmlerror.cc
	tinyxmlparser.cc
	Tracker.cc
	Utils.cc
	MathUtils.cc
	MKProtocol.cc
	MKConnection.cc
	MikroKopter.cc
	rs232.c
	VideoSource.cc
	${VIDEO_SOURCE_SRC}
)

add_executable(PTAML ${PTAML_SRC})
target_link_libraries(PTAML ${PTAML_LIBS})
target_link_libraries(PTAML flycapture)

