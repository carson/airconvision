This is a modified variant of PTAM that implements logging and marker-based scaling.

Installation instructions on Ubuntu 13.04
-----------------------------------------

# Install prerequisites:

sudo apt-get update
sudo apt-get install build-essential libpng12-dev libblas-dev liblapack-dev freeglut3-dev libreadline-dev libgstreamer0.10-dev libgstreamer-plugins-base0.10-dev libv4l-dev libxi-dev libxmu-dev libopencv-dev libboost-signals-dev cmake git cvs

# make project directory called airconvision

mkdir /opt/airconvision

# Install TooN:

cd /opt/airconvision
git clone https://github.com/edrosten/TooN.git
cd TooN
./configure
sudo make install
cd ..

# Install libCVD:

git clone https://github.com/edrosten/libcvd.git
cd libcvd
export CXXFLAGS=-D_REENTRANT
./configure --without-ffmpeg
sudo make -j4
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

cd /tmp
wget http://downloads.sourceforge.net/project/artoolkit/artoolkit/2.72.1/ARToolKit-2.72.1.tgz
tar -xvf ARToolKit-2.72.1.tgz
sudo mv ARToolKit /opt
cd /opt/ARToolKit
./Configure
# select Video4Linux
# do not use color conversion with x86 assembly
# create debug symbols
# build gsub libraries with texture rectangle support
make -j4
cd ..

# Install flycapture
cd /tmp
# download flycapture from pointgrey research
tar -xvf flycapture2-2.4.3.10-amd64-pkg.tar
cp -R /tmp/flycapture2-2.4.3.10-amd64 /opt/flycapture2-2.4.3.10-amd64
cd /opt/flycapture2-2.4.3.10-amd64
sudo apt-get install -f libglade2-0 libglademm-2.4-1c2a libgtkmm-2.4-1c2a libgtkglextmm-x11-1.2-dev libgtkglext1 libgtkglext1-dev libgtkglextmm-x11-1.2-0
# start installation process
sudo chmod +x install_flycapture.sh
./install_flycapture.sh
# setup flycapture user

# get and build PTAML

cd /opt/airconvision
# choose flycapture branch
git clone https://github.com/carson/airconvision/tree/flycapture
# clean up nested directories called airconvision
mv airconvision/PTAML . && rm -rf airconvision
cd PTAML
mkdir BUILD
cd BUILD
# specify the correct directory for the ARToolKit
cmake ..
sudo ldconfig
make -j4

cd ..
# configre settings
cp settings_template.cfg settings.cfg
# plug in usb camera
# run PTAML
./BUILD/bin/PTAML
