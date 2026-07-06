#include <signal.h>
#include <unistd.h>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"

volatile sig_atomic_t interrupted = false;
void signal_end(int) { interrupted = true; }

static void captureFrames(oc::ArgusBayerCapture& cam, cv::Mat& frame, int n) {
  int channels = cam.getNumberOfChannels();
  size_t nbytes =
      static_cast<size_t>(cam.getWidth()) * cam.getHeight() * channels;
  for (int i = 0; i < n && !interrupted; i++) {
    while (!interrupted && !cam.isNewFrame()) usleep(100);
    std::memcpy(frame.data, cam.getPixels(), nbytes);
  }
}

int main(int argc, char* argv[]) {
  struct sigaction sa = {};
  sa.sa_handler = signal_end;
  sigaction(SIGINT, &sa, nullptr);

  int camera_id = 0;
  int settle = 15;
  if (argc > 1) camera_id = atoi(argv[1]);
  if (argc > 2) settle = atoi(argv[2]);

  oc::ArgusCameraConfig config;
  config.mDeviceId = camera_id;
  config.mWidth = 1920;
  config.mHeight = 1080;
  config.mFPS = 15;
  config.hdr = false;
  config.mSwapRB = true;
  config.verbose_level = 1;

  oc::ArgusBayerCapture cam;
  auto state = cam.openCamera(config);
  if (state != oc::ARGUS_STATE::OK) {
    std::cerr << "Failed to open camera: " << oc::ARGUS_STATE2str(state)
              << std::endl;
    return 1;
  }

  cv::Mat frame(cam.getHeight(), cam.getWidth(), CV_8UC4);

  // Initial settle
  std::cout << "Settling " << settle << " frames..." << std::endl;
  captureFrames(cam, frame, settle);

  std::string outdir = "/tmp/wb_test";
  std::string mkdir_cmd = "mkdir -p " + outdir;
  system(mkdir_cmd.c_str());

  auto saveFrame = [&](const std::string& name) {
    cv::Mat bgr;
    cv::cvtColor(frame, bgr, cv::COLOR_RGBA2BGR);
    std::string path = outdir + "/" + name + ".png";
    cv::imwrite(path, bgr);
    std::cout << "  Saved " << path << std::endl;
  };

  auto printGains = [&]() {
    float g[4];
    cam.getAwbGains(g);
    std::cout << "  AWB gains: R=" << g[0] << " Ge=" << g[1]
              << " Go=" << g[2] << " B=" << g[3] << std::endl;
  };

  // Test auto WB as baseline
  std::cout << "\n=== Auto WB ===" << std::endl;
  cam.setAutomaticWhiteBalance(1);
  captureFrames(cam, frame, settle);
  std::cout << "  WB status=" << cam.getAutomaticWhiteBalanceStatus()
            << " temp=" << cam.getManualWhiteBalance() << "K" << std::endl;
  printGains();
  saveFrame("auto");

  // Test preset AWB modes
  std::vector<std::pair<int, std::string>> awb_modes = {
      {2, "incandescent"},   {3, "fluorescent"},
      {4, "warm_fluor"},     {5, "daylight"},
      {6, "cloudy_daylight"}};
  for (auto& [mode, name] : awb_modes) {
    std::cout << "\n=== AWB mode " << mode << " (" << name << ") ==="
              << std::endl;
    cam.setAutomaticWhiteBalance(mode);
    captureFrames(cam, frame, settle);
    std::cout << "  WB status=" << cam.getAutomaticWhiteBalanceStatus()
              << " temp=" << cam.getManualWhiteBalance() << "K" << std::endl;
    printGains();
    saveFrame("awb_" + name);
  }

  // Sweep manual temps
  std::vector<uint32_t> temps = {2800, 3200, 3500, 4000, 4500,
                                 5000, 5500, 6000, 6500, 8000};
  for (uint32_t t : temps) {
    if (interrupted) break;
    std::cout << "\n=== Manual " << t << "K ===" << std::endl;
    int rc = cam.setManualWhiteBalance(t);
    std::cout << "  setManualWhiteBalance(" << t << ") -> " << rc << std::endl;
    captureFrames(cam, frame, settle);
    std::cout << "  WB status=" << cam.getAutomaticWhiteBalanceStatus()
              << " temp=" << cam.getManualWhiteBalance() << "K" << std::endl;
    printGains();
    saveFrame("manual_" + std::to_string(t) + "K");
  }

  // ================================================================
  // Exposure sweep - restore auto WB first for consistent comparison
  // ================================================================
  std::cout << "\n\n======== EXPOSURE SWEEP ========" << std::endl;
  cam.setAutomaticWhiteBalance(1);
  captureFrames(cam, frame, settle);

  auto printExposure = [&]() {
    std::cout << "  Exposure: " << cam.getFrameExposureTime() << " us ("
              << cam.getFrameExposurePercent() << "%)"
              << "  Analog gain: " << cam.getAnalogFrameGain() << " dB"
              << "  Digital gain: " << cam.getDigitalFrameGain() << std::endl;
  };

  // Auto exposure baseline
  std::cout << "\n=== Auto Exposure (baseline) ===" << std::endl;
  cam.setAutomaticExposure();
  captureFrames(cam, frame, settle);
  printExposure();
  saveFrame("exp_auto");

  // Exposure range clamping
  std::vector<std::pair<uint64_t, uint64_t>> exp_ranges = {
      {16, 5000}, {16, 10000}, {16, 20000}, {16, 33000}, {16, 39000}};
  for (auto& [lo, hi] : exp_ranges) {
    if (interrupted) break;
    std::cout << "\n=== Exposure range " << lo << "-" << hi << " us ==="
              << std::endl;
    cam.setAutomaticExposure();
    cam.setFrameExposureRange(lo, hi);
    captureFrames(cam, frame, settle);
    printExposure();
    uint64_t rlo = 0, rhi = 0;
    cam.getFrameExposureRange(rlo, rhi);
    std::cout << "  Readback range: " << rlo << " - " << rhi << " us"
              << std::endl;
    saveFrame("exp_range_" + std::to_string(lo) + "_" + std::to_string(hi));
  }

  // Manual exposure by percent
  std::vector<int> exp_pcts = {10, 25, 50, 75, 100};
  for (int pct : exp_pcts) {
    if (interrupted) break;
    std::cout << "\n=== Manual Exposure " << pct << "% ===" << std::endl;
    int rc = cam.setManualExposure(pct);
    std::cout << "  setManualExposure(" << pct << ") -> " << rc << std::endl;
    captureFrames(cam, frame, settle);
    printExposure();
    saveFrame("exp_manual_" + std::to_string(pct) + "pct");
  }

  // Manual exposure by time
  std::vector<uint64_t> exp_times = {1000, 5000, 10000, 20000, 33000};
  for (uint64_t t : exp_times) {
    if (interrupted) break;
    std::cout << "\n=== Manual Exposure " << t << " us ===" << std::endl;
    int rc = cam.setManualTimeExposure(t);
    std::cout << "  setManualTimeExposure(" << t << ") -> " << rc << std::endl;
    captureFrames(cam, frame, settle);
    printExposure();
    saveFrame("exp_time_" + std::to_string(t) + "us");
  }

  // EV compensation
  cam.setAutomaticExposure();
  cam.setFrameExposureRange(16, 39000);  // Reset range
  std::vector<float> ev_values = {-2.0, -1.0, -0.5, 0.0, 0.5, 1.0, 2.0};
  for (float ev : ev_values) {
    if (interrupted) break;
    std::cout << "\n=== EV Compensation " << ev << " ===" << std::endl;
    int rc = cam.setExposureCompensation(ev);
    std::cout << "  setExposureCompensation(" << ev << ") -> " << rc
              << std::endl;
    captureFrames(cam, frame, settle);
    printExposure();
    saveFrame("exp_ev_" + std::to_string(static_cast<int>(ev * 10)));
  }

  // Analog gain modes
  std::cout << "\n\n======== GAIN SWEEP ========" << std::endl;
  cam.setAutomaticExposure();
  cam.setExposureCompensation(0.0);

  std::cout << "\n=== Auto Analog Gain ===" << std::endl;
  cam.setAutomaticAnalogGain();
  captureFrames(cam, frame, settle);
  printExposure();
  saveFrame("gain_analog_auto");

  std::vector<float> ag_db = {0.1, 3.0, 6.0, 12.0, 20.0};
  for (float db : ag_db) {
    if (interrupted) break;
    std::cout << "\n=== Manual Analog Gain " << db << " dB ===" << std::endl;
    int rc = cam.setManualAnalogGainReal(db);
    std::cout << "  setManualAnalogGainReal(" << db << ") -> " << rc
              << std::endl;
    captureFrames(cam, frame, settle);
    printExposure();
    saveFrame("gain_analog_" + std::to_string(static_cast<int>(db * 10))
              + "db");
  }

  // Restore auto for digital gain test
  cam.setAutomaticAnalogGain();

  std::cout << "\n=== Auto Digital Gain ===" << std::endl;
  cam.setAutomaticDigitalGain();
  captureFrames(cam, frame, settle);
  printExposure();
  saveFrame("gain_digital_auto");

  std::vector<int> dg_factors = {1, 2, 4, 8};
  for (int f : dg_factors) {
    if (interrupted) break;
    std::cout << "\n=== Manual Digital Gain factor=" << f << " ==="
              << std::endl;
    int rc = cam.setManualDigitalGainReal(f);
    std::cout << "  setManualDigitalGainReal(" << f << ") -> " << rc
              << std::endl;
    captureFrames(cam, frame, settle);
    printExposure();
    saveFrame("gain_digital_" + std::to_string(f) + "x");
  }

  // Restore auto
  cam.setAutomaticDigitalGain();
  cam.setAutomaticAnalogGain();
  cam.setAutomaticExposure();

  std::cout << "\n=== Done. Images in " << outdir << " ===" << std::endl;
  cam.closeCamera();
  return 0;
}
