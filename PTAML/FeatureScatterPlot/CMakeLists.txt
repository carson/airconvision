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
    ../PTAML/agast/nonMaximumSuppression.cc
    ../PTAML/agast/agast5_8_nms.cc
    ../PTAML/agast/agast5_8.cc
    ../PTAML/agast/agast7_12d_nms.cc
    ../PTAML/agast/agast7_12d.cc
    ../PTAML/agast/agast7_12s_nms.cc
    ../PTAML/agast/agast7_12s.cc
    ../PTAML/agast/oast9_16_nms.cc
    ../PTAML/agast/oast9_16.cc
)

include_directories(
    ../PTAML
)

set(SCATTER_SRC
	../PTAML/ShiTomasi.cc
	../PTAML/Timing.cc
	scatter.cc
	${AGAST_SRC}
)

add_executable(Scatter ${SCATTER_SRC})
target_link_libraries(Scatter ${SCATTER_LIBS})

