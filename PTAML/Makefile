# DO NOT DELETE THIS LINE -- make depend depends on it.

#3DS modelling game dependencies
3DSLIB = -l3ds
3DSFILES =	ModelsGame.o\
		ModelBrowser.o\
		ModelControls.o\
		MGButton.o\
		Model3ds.o\
		ModelsGameData.o

EXTRA_CFLAGS:= -DENABLE_MODELS_GAME

## For Gstreamer
MY_CUSTOM_INCLUDE_PATH=`pkg-config gstreamer-0.10 --cflags`
MY_CUSTOM_LINK_PATH=`pkg-config gstreamer-0.10 --libs`

# Edit the lines below to point to any needed include and link paths
# Or to change the compiler's optimization flags
CC = g++
# You can add -pg to profile
DEBUGFLAGS = -g 
LINKFLAGS = $(MY_CUSTOM_LINK_PATH) -lGVars3 -lcvd $(3DSLIB) -lgstapp-0.10 -lglut -lGLU -llapack -lblas $(DEBUGFLAGS)
COMPILEFLAGS = $(MY_CUSTOM_INCLUDE_PATH) -D_LINUX -D_REENTRANT -Wall -O3 -march=core2 -msse3 -fno-strict-aliasing $(DEBUGFLAGS)

# Edit this line to change video source
#VIDEOSOURCE = VideoSource_Linux_DV.o
#VIDEOSOURCE = VideoSource_Linux_V4L.o # VideoSource_Linux_V4L.o 
VIDEOSOURCE = VideoSource_Linux_Gstreamer_File.o
#VIDEOSOURCE = VideoSource_Linux_Gstreamer_Camera.o

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
		EyeGame.o\
		Tracker.o\
		tinyxml.o\
		tinyxmlerror.o\
		tinyxmlparser.o\
		MapLockManager.o\
		MD5.o\
		MD5Wrapper.o\
		MapSerializer.o\
		Games.o\
		Utils.o\
		ShooterGame.o\
		ShooterGameTarget.o\
		$(3DSFILES)\
		TakFrame.h

CALIB_OBJECTS=	GLWindow2.o\
		GLWindowMenu.o\
		$(VIDEOSOURCE)\
		CalibImage.o \
		CalibCornerPatch.o\
		ATANCamera.o \
		CameraCalibrator.o

All: PTAML CameraCalibrator

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
