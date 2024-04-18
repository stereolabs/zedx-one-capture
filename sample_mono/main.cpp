#include <memory>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"

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


    std::vector<oc::ArgusDevice> devs = oc::ArgusBayerCapture::getArgusDevices();
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
    config.mDeviceId = (camera_id_0);
    config.mFPS = rq_fps;
    config.mWidth = rq_width;
    config.mHeight = rq_height;
    config.verbose_level = 3;

    /// Open the camera
    oc::ArgusBayerCapture camera_0;
    oc::ARGUS_STATE state_cam0 = camera_0.openCamera(config);
    if (state_cam0 != oc::ARGUS_STATE::OK) {
        std::cerr << "Failed to open Camera, error code " << ARGUS_STATE2str(state_cam0) << std::endl;
        return -1;
    }

    cv::Mat rgb_d, rgb_cam0;
    rgb_cam0 = cv::Mat(camera_0.getHeight(), camera_0.getWidth(), CV_8UC4, 1);

    std::cout << "Press 's' to save images" << std::endl;
    char key = ' ';
    int image_count = 0;
    while (key != 'q') {
      if (camera_0.isNewFrame()) {

        memcpy(rgb_cam0.data, camera_0.getPixels(),
               camera_0.getWidth() * camera_0.getHeight() *
                   camera_0.getNumberOfChannels());
        /// Only with argus ///
       // std::cout << " Exposure Time : " << camera_0.getFrameExposureTime()
       //           << " us" << std::endl;
       // std::cout << " Analog Gain : " << camera_0.getAnalogFrameGain() << " dB"
       //           << std::endl;
       // std::cout << " Digital Gain : " << camera_0.getDigitalFrameGain()
       //           << std::endl;
       ///////////////////

        cv::resize(rgb_cam0, rgb_d, cv::Size(1280, 720));
        cv::imshow("image", rgb_d);
        key = cv::waitKey(2);

        if (key == 's') {
          // saves the images
          cv::imwrite("image_left_" + std::to_string(image_count) + ".png",
                      rgb_cam0);
          std::cout << "images created." << std::endl;
          image_count++;
        }
      }
      else
        usleep(100);
    }
    camera_0.closeCamera();

    return 0;

}

