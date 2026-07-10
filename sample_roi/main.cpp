// Copyright 2026 Skyways
//
// Standalone ISP source-clip (ROI) smoke test for ZED X One mono capture.
// Opens the camera with a configurable clip rect, grabs a frame after a short
// warmup, and writes it to a PNG so the crop/zoom can be eyeballed without ROS
// or a camera_info publisher. Headless (no imshow) so it runs over SSH.
//
// Usage:
//   argus_roi_test <device> <out_w> <out_h> <fps> \
//                  <roi_x> <roi_y> <roi_w> <roi_h> \
//                  [sensor_w] [sensor_h] [warmup_frames]
//
// ROI is in SENSOR (source) pixels; the clip is normalized over the sensor mode.
// sensor_w/sensor_h default to the output size (source == output => the ISP clip
// is a digital zoom). Set the sensor larger than the output for a true 1:1 crop,
// e.g. out 1920x1080, sensor 3840x2160, roi 960 540 1920 1080 -> centered 1080
// crop of the 4K sensor at native detail (scale = 1.0, no zoom).
// roi_w == 0 captures the full frame. Each run writes
// roi_x<X>_y<Y>_w<W>_h<H>_s<SW>x<SH>.png.

#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <cstring>
#include <iostream>

#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"

namespace {
float clamp01(float v) { return std::min(std::max(v, 0.f), 1.f); }
}  // namespace

int main(int argc, char* argv[]) {
  int device = 0, out_w = 1920, out_h = 1080, fps = 15;
  int roi_x = 0, roi_y = 0, roi_w = 0, roi_h = 0;
  int sensor_w = 0, sensor_h = 0, warmup = 15;
  if (argc > 1) device = atoi(argv[1]);
  if (argc > 2) out_w = atoi(argv[2]);
  if (argc > 3) out_h = atoi(argv[3]);
  if (argc > 4) fps = atoi(argv[4]);
  if (argc > 5) roi_x = atoi(argv[5]);
  if (argc > 6) roi_y = atoi(argv[6]);
  if (argc > 7) roi_w = atoi(argv[7]);
  if (argc > 8) roi_h = atoi(argv[8]);
  if (argc > 9) sensor_w = atoi(argv[9]);
  if (argc > 10) sensor_h = atoi(argv[10]);
  if (argc > 11) warmup = atoi(argv[11]);

  // Sensor (source) frame the clip is normalized over. Default: source==output.
  const int src_w = sensor_w > 0 ? sensor_w : out_w;
  const int src_h = sensor_h > 0 ? sensor_h : out_h;

  // Normalized clip rect over the sensor frame (mirrors ToNormalizedClipRect).
  float cl = 0.f, ct = 0.f, cr = 1.f, cb = 1.f;
  const bool roi_on = roi_w > 0 && roi_h > 0 && src_w > 0 && src_h > 0;
  if (roi_on) {
    cl = clamp01(static_cast<float>(roi_x) / src_w);
    ct = clamp01(static_cast<float>(roi_y) / src_h);
    cr = clamp01(static_cast<float>(roi_x + roi_w) / src_w);
    cb = clamp01(static_cast<float>(roi_y + roi_h) / src_h);
  }
  // scale = output / clipped-source-pixels. 1.0 => true 1:1 crop; >1 => zoom.
  const float clip_px_w = (cr - cl) * src_w;
  const float scale = clip_px_w > 0.f ? out_w / clip_px_w : 0.f;
  std::cout << "ROI " << (roi_on ? "ENABLED" : "disabled") << " x=" << roi_x
            << " y=" << roi_y << " w=" << roi_w << " h=" << roi_h << " over "
            << "sensor " << src_w << "x" << src_h << " -> clip [" << cl << ", "
            << ct << ", " << cr << ", " << cb << "]  output " << out_w << "x"
            << out_h << "  scale=" << scale << "x ("
            << (std::abs(scale - 1.f) < 0.01f ? "true 1:1 crop" : "zoom") << ")"
            << std::endl;

  oc::ArgusCameraConfig config;
  config.mDeviceId = device;
  config.mWidth = out_w;
  config.mHeight = out_h;
  config.mSensorWidth = sensor_w;   // 0 => sensor mode == output
  config.mSensorHeight = sensor_h;
  config.mFPS = fps;
  config.verbose_level = 1;
  config.mClipLeft = cl;
  config.mClipTop = ct;
  config.mClipRight = cr;
  config.mClipBottom = cb;

  oc::ArgusBayerCapture camera;
  oc::ARGUS_STATE state = camera.openCamera(config);
  if (state != oc::ARGUS_STATE::OK) {
    // Invalid clip or unavailable sensor mode -> INVALID_SOURCE_CONFIGURATION.
    std::cerr << "openCamera failed: " << oc::ARGUS_STATE2str(state) << std::endl;
    return 1;
  }

  cv::Mat rgba(camera.getHeight(), camera.getWidth(), CV_8UC4);
  int got = 0;
  for (int i = 0; i < warmup + 1;) {
    if (camera.isNewFrame()) {
      std::memcpy(rgba.data, camera.getPixels(),
                  static_cast<size_t>(camera.getWidth()) * camera.getHeight() *
                      camera.getNumberOfChannels());
      got = ++i;
    } else {
      usleep(1000);
    }
  }
  camera.closeCamera();
  if (got == 0) {
    std::cerr << "No frames captured" << std::endl;
    return 2;
  }

  cv::Mat bgr;
  cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
  char name[160];
  snprintf(name, sizeof(name), "roi_x%d_y%d_w%d_h%d_s%dx%d.png", roi_x, roi_y,
           roi_w, roi_h, src_w, src_h);
  cv::imwrite(name, bgr);
  std::cout << "Wrote " << name << " (" << bgr.cols << "x" << bgr.rows << ")"
            << std::endl;
  return 0;
}
