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

## ISP source-clip (ROI crop / zoom)

`oc::ArgusCameraConfig` supports a normalized ISP source-clip rectangle, letting
the camera crop and rescale the sensor image on-chip before it reaches your
application:

- `mClipLeft` / `mClipTop` / `mClipRight` / `mClipBottom` — a normalized `[0,1]`
  clip rectangle over the sensor (source) frame (full frame = `{0, 0, 1, 1}`).
  The ISP clips the source to this rectangle and rescales it to the output
  resolution `mWidth` × `mHeight`.
- `mSensorWidth` / `mSensorHeight` — the sensor-mode (source readout) resolution.
  Leave at `0` to use `mWidth` / `mHeight` (source == output, so a clip is a
  digital zoom). Set larger than the output to read the sensor at higher
  resolution, so a clip yields a true 1:1 crop at native detail instead of a
  zoom (e.g. sensor `3840×2160`, output `1920×1080`, centered clip → native
  center crop).

An out-of-range or inverted clip rectangle causes `openCamera()` to fail with
`INVALID_SOURCE_CONFIGURATION`.

See [`sample_roi`](sample_roi/) for a standalone tool that exercises this
feature.
