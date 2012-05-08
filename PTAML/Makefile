# DO NOT DELETE THIS LINE -- make depend depends on it.

# OpenCV build of PTAMM by carson 15-FEB-2012

#3DS modelling game dependencies

EXTRA_CFLAGS:= 

## For Gstreamer
MY_CUSTOM_INCLUDE_PATH=-I/home/daniel/Code/ARToolKit/include
MY_CUSTOM_LINK_PATH=-L/home/daniel/Code/ARToolKit/lib

# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++
LINKFLAGS = $(MY_CUSTOM_LINK_PATH) -lGVars3 -lcvd -lgstapp-0.10 -lARgsub -lAR -lglut -lGLU -lGL -llapack -lblas -g
COMPILEFLAGS = $(MY_CUSTOM_INCLUDE_PATH) -D_LINUX -D_REENTRANT -Wall -O3 -march=core2 -msse3 -fno-strict-aliasing -std=gnu++0x -g

# Edit this line to change video source
#VIDEOSOURCE = VideoSource_Linux_DV.o
VIDEOSOURCE = VideoSource_Linux_V4L.o
#VIDEOSOURCE = VideoSource_Linux_Gstreamer_File.o
#VIDEOSOURCE = VideoSource_Linux_Gstreamer_Camera.o
#VIDEOSOURCE = VideoSource_Linux_OpenCV.o

OBJECTS=	main.o\
		GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		System.o \
		ATANCamera.o\
		KeyFrame.o\
		MapPoint.o\
		Map.o\
		SmallBlurryImage.o\
		ShiTomasi.o \
		HomographyInit.o \
		MapMaker.o \
		Bundle.o \
		PatchFinder.o\
		Relocaliser.o\
		MiniPatch.o\
		MapViewer.o\
		ARDriver.o\
		Tracker.o\
		tinyxml.o\
		tinyxmlerror.o\
		tinyxmlparser.o\
		MapLockManager.o\
		MD5.o\
		MD5Wrapper.o\
		MapSerializer.o\
		Utils.o\
		ARToolkit.o\
		TakFrame.h

CALIB_OBJECTS=	GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		CalibImage.o \
		CalibCornerPatch.o\
		ATANCamera.o \
		CameraCalibrator.o

all: PTAML CameraCalibrator

PTAML: $(OBJECTS)
	$(CC) -o PTAML $(OBJECTS) $(LINKFLAGS)

CameraCalibrator:$(CALIB_OBJECTS)
	$(CC) -o CameraCalibrator $(CALIB_OBJECTS) $(LINKFLAGS)

%.o: %.cc
	$(CC) $< -o $@ -c $(COMPILEFLAGS) $(EXTRA_CFLAGS)

clean:
	rm *.o

depend:
	rm dependecies; touch dependencies
	makedepend -fdependencies $(INCLUDEFLAGS) $(MOREINCS) *.cc *.h 

-include dependencies









