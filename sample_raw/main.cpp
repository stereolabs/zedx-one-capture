#include <memory>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"
#include <cstdlib>


int main(int argc, char *argv[]) {

    int camera_id_0 = 0;
    int rq_width=0;
    int rq_height=0;
    int rq_fps = 0;

    if (argc > 1) camera_id_0 = atoi(argv[1]);
    if (argc > 2) rq_width = atoi(argv[2]);
    if (argc > 3) rq_height = atoi(argv[3]);
    if (argc > 4) rq_fps = atoi(argv[4]);

    int major, minor, patch;
    oc::ArgusVirtualCapture::getVersion(major, minor, patch);
    std::cout << " Argus Capture Version : " << major << "." << minor << "." << patch << std::endl;

    std::vector<oc::ArgusDevice> devs = oc::ArgusV4l2Capture::getV4l2Devices();
    for (int i = 0; i < devs.size(); i++) {
        std::cout << "##################" << std::endl;
        std::cout << " Device : " << devs.at(i).id << std::endl;
        std::cout << " Name : " << devs.at(i).name << std::endl;
        std::cout << " Badge : " << devs.at(i).badge << std::endl;
        std::cout << " Available : " << devs.at(i).available << std::endl;
    }
    std::cout << "***********************" << std::endl;

    /// Create configuration for the camera
    oc::ArgusCameraConfig config;
    config.mDeviceId = camera_id_0;
    config.mFPS = rq_fps;
    config.mWidth = rq_width;
    config.mHeight = rq_height;
    config.mode = oc::PixelMode::RAW10; //RAW12 for 4K
    config.verbose_level = 2;

    /// Open the camera
    oc::ArgusV4l2Capture cam_raw;
    oc::ARGUS_STATE state_cam0 = cam_raw.openCamera(config);
    if (state_cam0 != oc::ARGUS_STATE::OK) {
        std::cerr << "Failed to open Camera, error code " << ARGUS_STATE2str(state_cam0) << std::endl;
        return -1;
    }

    std::cout<<" Creating Mat at resolution : "<<cam_raw.getHeight()<<cam_raw.getWidth()<<" PD : "<<cam_raw.getPixelDepth()<<std::endl;
    cv::Mat rgb_cam0 = cv::Mat(cam_raw.getHeight(), cam_raw.getWidth(), CV_16UC1, 1);

    char key = ' ';
    while (key != 'q') {
      if (cam_raw.isNewFrame()) {

        memcpy(rgb_cam0.data, cam_raw.getPixels(),
               cam_raw.getWidth() * cam_raw.getHeight() *
                   cam_raw.getNumberOfChannels()* cam_raw.getPixelDepth());
        cv::imshow("Image RAW", rgb_cam0);
        key = cv::waitKey(2);
      }
      else
        usleep(100);

    }
    return 0;
}

