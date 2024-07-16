# ZED X One Stereo Calibration

This sample show how to calibrate 2 ZED X one mono-cameras in stereo configuration with a custom baseline to be able to use it in the ZED SDK.

A calibration pattern is required, the bigger, the better. As a starting point a small printed chessboard (A4) can be used, make sure it's flat and that the dimension and characteristics are correctly [entered in the code](https://github.com/stereolabs/zedx-one-capture/blob/6c5254b728c5f683db373a1f4d66abead930b27f/stereo_calibration/main.cpp#L13-L15).

![Opencv Checkerboard](checkerboard_sample.png)

Other chessboard can be generated using OpenCV scripts https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html

Before starting the calibration be sure to customize the code according to the type of calibration board you are using:

```C++
// CHANGE THIS PARAM BASED ON THE CHECKERBOARD USED
// https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html
int target_w = 9; // number of inner squares
int target_h = 6;
float square_size = 25.0; // mm
```

> **Note:*** the target Width and the target Height corresponds to the number of inner edges, not to the number of outer squares.

## Perform the calibration

The calibration process uses images saved manually with the chessboard visible and at different position each time in the image.

When calibrating, make sure the calibration pattern is seen in all the field of view of the camera along the process.

> Note: if the calibration tool seems frozen, just wait for a few seconds that the elaboration is completed. We recommend using the `MAXN` power mode 
and run the `jetson_clocks` script to boost the elaborations.

### Step 1: move the chessboard

Move the chessboard and hit the `s` key to save images. Each time you save a frame, the tool calculates the are of the image covered by the calibration board
and highlight it with a "green zone". When the left image is fully covered by green, then the step 1 is completed.

### Step 2: rotate the chessboard

Move the chessboard away/close to the camera and rotate it around the three axis.

The chessboard should be tilted along each axis at different distances, the GUI indicates which axis should be further excited.

![Camera Axis](CoordinateSystem.jpg)

When all the axis are fully excited, the calibration process starts automatically.

> **WARNING**: never rotate the chessboard around the Z axis with an angle bigger than 60°, otherwise the algorithm will wrongly detect the chessboard in portrait mode instead of landscape.

### Step 3: calibration processing

Once enough images are saved the calibration process will run and output a conf file readable by the SDK. 

It should be renamed using the virtual serial number chosen in the ZED Media Server tool. 

See https://www.stereolabs.com/docs/get-started-with-zed-x-one/zed-x-one-stereo/ for more information.

### Video Tutorial

This video tutorial illustrates how to perform all the steps to obtain a valid calibration.

[![Video Tutorial](https://img.youtube.com/vi/Dd-4_eUdkSM/0.jpg)](https://www.youtube.com/watch?v=Dd-4_eUdkSM)
