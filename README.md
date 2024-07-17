# ZED X One Capture

This repository provides a C++ library that serves as a camera control API for Stereolabs' [ZED X One GMSL2 cameras](https://www.stereolabs.com/products/zed-x-one). 

Designed specifically for use with NVIDIA Jetson platforms, this library empowers you to interact with and control your [ZED X One camera](https://store.stereolabs.com/en-it/products/zed-x-one). 

To ensure proper functionality, make sure you have the [correct drivers installed](https://www.stereolabs.com/docs/get-started-with-zed-link). 

Refer to the official [ZED X One getting started guide](https://www.stereolabs.com/docs/get-started-with-zed-x-one) for detailed instructions.

## Usage

Open a terminal (`Ctrl+Alt+t`) and clone this repository

```
git clone https://github.com/stereolabs/zedx-one-capture.git
```

Build and install the C++ library in the `lib` folder

```
cd zedx-one-capture/lib
mkdir build; cd build; cmake ..
make
sudo make install
sudo ldconfig
```

Compile the sample

```
cd ../sample_mono/
mkdir build; cd build; cmake ..
make
```

Run the sample

```
./argus_camera
```
