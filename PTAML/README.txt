This is a modified variant of PTAM that implements logging and marker-based scaling.

Installation instructions on Ubuntu 12.10
-----------------------------------------

# Install prerequisites:

sudo apt-get update
sudo apt-get install build-essential libpng12-dev libblas-dev liblapack-dev freeglut3-dev libreadline-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libxi-dev libxmu-dev libopencv-dev libboost-signals-dev cmake git cvs

# make project directory called airconvision

mkdir airconvision

# Install TooN:

cd airconvision
git clone https://github.com/edrosten/TooN.git
cd TooN
./configure
sudo make install
cd ..

# Install libCVD:

git clone git://git.savannah.nongnu.org/libcvd.git
cd libcvd
export CXXFLAGS=-D_REENTRANT
./configure --without-ffmpeg
make -j4
sudo make install
cd ..

# Install GVars3:

git clone https://github.com/edrosten/gvars.git
cd gvars
./configure --disable-widgets
make -j4
sudo make install

# Install ARToolKit:

# Create a symbolic link to videodev.h:
# Note: This might not be best practice but I found it to be the
# easiest way to get everything compiling without modifying a lot of
# code.
sudo ln -s /usr/include/libv4l1-videodev.h /usr/include/linux/videodev.h

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

#Get and build PTAML

git clone git clone https://github.com/carson/airconvision.git
# clean up nested directories called airconvision
cd airconvision
mv PTAML ..
cd ..
rm -rf airconvision
cd PTAML
mkdir BUILD
cd BUILD
# specify the correct directory for the ARToolKit
cmake -D ARTOOLKIT_DIR=../../../ARToolKit ..
sudo ldconfig
make -j4

cd ..
# plug in usb camera
# run PTAML
./BUILD/bin/PTAML

