# Libraries
set(SCATTER_LIBS
	${GSTREAMER_LIBRARIES}
	gstapp-0.10
	GVars3
	cvd
	GLU 
	GL
	lapack
	blas
)

set(AGAST_SRC
    ../src/agast/nonMaximumSuppression.cc
    ../src/agast/agast5_8_nms.cc
    ../src/agast/agast5_8.cc
    ../src/agast/agast7_12d_nms.cc
    ../src/agast/agast7_12d.cc
    ../src/agast/agast7_12s_nms.cc
    ../src/agast/agast7_12s.cc
    ../src/agast/oast9_16_nms.cc
    ../src/agast/oast9_16.cc
)

include_directories(
    ../src
)

set(SCATTER_SRC
	../src/ShiTomasi.cc
	../src/Timing.cc
	scatter.cc
	${AGAST_SRC}
)

add_executable(Scatter ${SCATTER_SRC})
target_link_libraries(Scatter ${SCATTER_LIBS})

