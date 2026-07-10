# wb_test

A local validation tool for the ZED X One capture library. It opens a single
camera and sweeps through the white balance, exposure, and gain controls
exposed by `oc::ArgusBayerCapture`, printing the reported state at each step and
saving a sample image so the results can be compared visually.

This is a developer diagnostic tool — it is **not** part of the public samples
and is not intended for distribution.

## What it does

For each stage the tool waits for the settings to settle, prints the values read
back from the camera, and writes a PNG to `/tmp/wb_test/`:

- **White balance** — auto WB baseline, the preset AWB modes (incandescent,
  fluorescent, warm fluorescent, daylight, cloudy daylight), and a manual color
  temperature sweep (2800 K – 8000 K).
- **Exposure** — auto baseline, exposure-range clamping, manual exposure by
  percent and by time, and EV compensation (−2 to +2).
- **Gain** — auto analog gain, a manual analog gain sweep in dB, auto digital
  gain, and a manual digital gain sweep (1×–8×).

At the end it restores automatic exposure, gain, and white balance before
closing the camera.

## Building and running

The `build_and_run.sh` helper builds the parent library in `../lib`, builds this
tool, and runs it:

```
./build_and_run.sh [camera_id] [settle_frames]
```

- `camera_id` — device index to open (default `0`).
- `settle_frames` — number of frames to capture between each setting change so
  the sensor can stabilize (default `15`).

To build manually instead:

```
mkdir build; cd build; cmake ..
make
export LD_LIBRARY_PATH="../../lib/build:$LD_LIBRARY_PATH"
./wb_test 0 15
```

## Requirements

- An NVIDIA Jetson with the ZED X One drivers installed (see the
  [getting started guide](https://www.stereolabs.com/docs/get-started-with-zed-x-one)).
- OpenCV (used to convert and save the sample frames).
- The Argus capture library built in `../lib/build`.

## Output

Sample images are written to `/tmp/wb_test/`, named after each stage (for
example `auto.png`, `awb_daylight.png`, `manual_5000K.png`, `exp_manual_50pct.png`,
`gain_analog_60db.png`). Press `Ctrl+C` at any time to stop early; the current
sweep is interrupted and the camera is closed cleanly.
