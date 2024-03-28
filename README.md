# ZED X One Capture

This repository provides a camera control API for ZED X One GMSL2 cameras from Stereolabs.

This runs on Nvidia Jetson.


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
cd ../sample/
mkdir build; cd build; cmake ..
make
./argus_camera
```