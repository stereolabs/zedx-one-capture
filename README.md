# ZED X One Capture

This repository provides a camera control API for ZED X One GMSL2 cameras from Stereolabs.

This runs on Nvidia Jetson. The correct drivers are required to open the camera, the documentation is available here https://www.stereolabs.com/docs/get-started-with-zed-x-one/


## Usage

Compile the C++ library in the lib folder;

```
git clone https://github.com/stereolabs/zedx-one-capture.git
cd zedx-one-capture/lib
mkdir build; cd build; cmake ..
make
sudo make install
```

Then compile the sample;

```
cd ../sample_mono/
mkdir build; cd build; cmake ..
make
./argus_camera
```
