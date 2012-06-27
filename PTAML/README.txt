This is a modified variant of PTAM that implements logging and marker-based scaling.

Installation instructions on Ubuntu 12.04
-----------------------------------------

# Install prerequisites:

sudo apt-get update
sudo apt-get install build-essential libblas-dev liblapack-dev freeglut3-dev libreadline-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libxi-dev libxmu-dev cmake git cvs

# make project directory called airconvision

mkdir airconvision

# Create a symbolic link to videodev.h:

# Note: This might not be best practice but I found it to be the
# easiest way to get everything compiling without modifying a lot of
# code.

sudo ln -s /usr/include/libv4l1-videodev.h /usr/include/linux/videodev.h

# Install TooN:

cd airconvision
git clone git://git.savannah.nongnu.org/toon.git
cd toon
./configure
sudo make install
cd ..

# Install libCVD:

git clone git://git.savannah.nongnu.org/libcvd.git
cd libcvd
export CXXFLAGS=-D_REENTRANT
./configure --without-ffmpeg

# Open the file "Makefile" and insert the following addition after line 113:
# Begin addition
                        cvd_src/Linux/v4l1buffer.o                      \
# End addition

make -j4
sudo make install
cd ..

# Install GVars3:

cvs -z3 -d:pserver:anoncvs@cvs.savannah.nongnu.org:/cvsroot/libcvd co gvars3
cd gvars3
./configure --disable-widgets
make -j4
sudo make install

# Install ARToolKit:

wget http://downloads.sourceforge.net/project/artoolkit/artoolkit/2.72.1/ARToolKit-2.72.1.tgz
tar -xvf ARToolKit-2.72.1.tgz
cd ARToolKit
./Configure
# select Video4Linux
# do not use color conversion with x86 assembly
# create debug symbols
# build gsub libraries with texture rectangle support
make -j4
cd ..

git clone git@github.com:carson/airconvision.git
cd airconvisio*
mkdir BUILD
cd BUILD
# specify the correct directory for the ARToolKit
cmake -D ARTOOLKIT_DIR=../ARToolKit ..
make -j4
