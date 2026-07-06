// Copyright 2026 Skyways
//
// Standalone ISP source-clip (ROI) smoke test for ZED X One mono capture.
// Opens the camera with a configurable clip rect, grabs a frame after a short
// warmup, and writes it to a PNG so the crop/zoom can be eyeballed without ROS
// or a camera_info publisher. Headless (no imshow) so it runs over SSH.
//
// Usage:
//   argus_roi_test <device> <width> <height> <fps> \
//                  <roi_x> <roi_y> <roi_w> <roi_h> [warmup_frames]
//
// ROI is in full-frame pixels, matching zedxone4k_camera_node's roi_* params.
// roi_w == 0 (default) captures the full frame. Each run writes
// roi_x<X>_y<Y>_w<W>_h<H>.png; run once full-frame and once cropped and diff.

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
  int device = 0, width = 1920, height = 1080, fps = 15;
  int roi_x = 0, roi_y = 0, roi_w = 0, roi_h = 0, warmup = 15;
  if (argc > 1) device = atoi(argv[1]);
  if (argc > 2) width = atoi(argv[2]);
  if (argc > 3) height = atoi(argv[3]);
  if (argc > 4) fps = atoi(argv[4]);
  if (argc > 5) roi_x = atoi(argv[5]);
  if (argc > 6) roi_y = atoi(argv[6]);
  if (argc > 7) roi_w = atoi(argv[7]);
  if (argc > 8) roi_h = atoi(argv[8]);
  if (argc > 9) warmup = atoi(argv[9]);

  // Normalized clip rect (mirrors ToNormalizedClipRect in the ROS node).
  float cl = 0.f, ct = 0.f, cr = 1.f, cb = 1.f;
  const bool roi_on = roi_w > 0 && roi_h > 0 && width > 0 && height > 0;
  if (roi_on) {
    cl = clamp01(static_cast<float>(roi_x) / width);
    ct = clamp01(static_cast<float>(roi_y) / height);
    cr = clamp01(static_cast<float>(roi_x + roi_w) / width);
    cb = clamp01(static_cast<float>(roi_y + roi_h) / height);
  }
  std::cout << "ROI " << (roi_on ? "ENABLED" : "disabled") << " x=" << roi_x
            << " y=" << roi_y << " w=" << roi_w << " h=" << roi_h
            << " -> clip [" << cl << ", " << ct << ", " << cr << ", " << cb
            << "]  output " << width << "x" << height << std::endl;

  oc::ArgusCameraConfig config;
  config.mDeviceId = device;
  config.mWidth = width;
  config.mHeight = height;
  config.mFPS = fps;
  config.verbose_level = 1;
  config.mClipLeft = cl;
  config.mClipTop = ct;
  config.mClipRight = cr;
  config.mClipBottom = cb;

  oc::ArgusBayerCapture camera;
  oc::ARGUS_STATE state = camera.openCamera(config);
  if (state != oc::ARGUS_STATE::OK) {
    // An invalid clip rect trips the new validation -> INVALID_SOURCE_CONFIGURATION.
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
  char name[128];
  snprintf(name, sizeof(name), "roi_x%d_y%d_w%d_h%d.png", roi_x, roi_y, roi_w,
           roi_h);
  cv::imwrite(name, bgr);
  std::cout << "Wrote " << name << " (" << bgr.cols << "x" << bgr.rows << ")"
            << std::endl;
  return 0;
}
