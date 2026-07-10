# sample_roi — ISP source-clip (ROI) smoke test

A standalone tool for validating the configurable **ISP source-clip rect** added
to the ZED X One capture library. It opens a single mono camera with a given
clip rectangle, grabs one frame after a short warmup, and writes it to a PNG so
the crop/zoom can be eyeballed. It is fully headless (no `imshow`), so it runs
over SSH.

## The feature

`oc::ArgusCameraConfig` gains two independent controls:

- **Source clip rect** — `mClipLeft` / `mClipTop` / `mClipRight` / `mClipBottom`,
  a normalized `[0,1]` rectangle over the sensor (source) frame. The full frame
  is `{0, 0, 1, 1}`. The ISP clips the source to this rectangle and rescales it
  to the output resolution (`mWidth` × `mHeight`). An invalid rect (out of
  `[0,1]`, or left ≥ right / top ≥ bottom) fails `openCamera` with
  `INVALID_SOURCE_CONFIGURATION`.

- **Sensor mode decoupled from output** — `mSensorWidth` / `mSensorHeight`.
  When `0`, the sensor mode is selected from `mWidth`/`mHeight` as before (source
  == output). When set larger than the output, the sensor is read at a higher
  resolution than the output stream, so a source-clip produces a **true 1:1
  crop** at native detail instead of a digital zoom.

Combining the two:

| Sensor mode | Output | Clip | Result |
|-------------|--------|------|--------|
| == output | 1920×1080 | centered half | digital **zoom** (upscaled) |
| 3840×2160 | 1920×1080 | centered 1920×1080 | native-detail **1:1 crop** |

## Usage

```
argus_roi_test <device> <out_w> <out_h> <fps> \
               <roi_x> <roi_y> <roi_w> <roi_h> \
               [sensor_w] [sensor_h] [warmup_frames]
```

- `device` — camera device index.
- `out_w` / `out_h` / `fps` — output stream resolution and frame rate.
- `roi_x` / `roi_y` / `roi_w` / `roi_h` — ROI in **sensor (source) pixels**. The
  tool normalizes it over the sensor mode into a clip rect. `roi_w == 0` (or
  `roi_h == 0`) captures the full frame.
- `sensor_w` / `sensor_h` — optional sensor-mode resolution. Defaults to the
  output size (source == output ⇒ the clip is a digital zoom). Set larger than
  the output for a true 1:1 crop.
- `warmup_frames` — frames to discard before saving (default `15`).

Each run prints the computed clip rect and scale factor, then writes
`roi_x<X>_y<Y>_w<W>_h<H>_s<SW>x<SH>.png`.

### Examples

Centered 1:1 crop of a 4K sensor (no zoom, `scale = 1.0`):

```
./argus_roi_test 0 1920 1080 15  960 540 1920 1080  3840 2160
```

2× centered digital zoom (source == output):

```
./argus_roi_test 0 1920 1080 15  480 270 960 540
```

Full frame (baseline):

```
./argus_roi_test 0 1920 1080 15  0 0 0 0
```

## Building

Build and install the library in `../lib` first (see the top-level README), then:

```
mkdir build; cd build; cmake ..
make
./argus_roi_test 0 1920 1080 15 0 0 0 0
```

## Requirements

- An NVIDIA Jetson with the ZED X One drivers installed
  ([getting started guide](https://www.stereolabs.com/docs/get-started-with-zed-x-one)).
- OpenCV (used to convert and save the sample frame).
- The Argus capture library from `../lib`.
