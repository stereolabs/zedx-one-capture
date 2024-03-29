#include <memory>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char *argv[]) {

    int camera_id_0 = 0;
    int camera_id_1 = 1;
    if (argc > 1) {
        camera_id_0 = atoi(argv[1]);
        camera_id_1 = atoi(argv[2]);
    }

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

    oc::ArgusCameraConfig config;
    config.mDeviceId = (camera_id_0);
    // for 4K 
#if 0
    config.mFPS = 15;
    config.mWidth = 3856;
    config.mHeight = 2180;
#else
    config.mFPS = 30;
    config.mWidth = 1920;
    config.mHeight = 1200;
#endif
    config.verbose_level = 3;

    oc::ArgusCameraConfig config_1;
    config_1.mDeviceId = (camera_id_1);
    config_1.mFPS = config.mFPS;
    config_1.mWidth = config.mWidth;
    config_1.mHeight = config.mHeight;
    config_1.verbose_level = config.verbose_level;

    oc::ArgusBayerCapture camera_0, camera_1;
    oc::ARGUS_STATE state = camera_0.openCamera(config);
    oc::ARGUS_STATE state_isx031 = camera_1.openCamera(config_1);


    if (state != oc::ARGUS_STATE::OK) {
        printf("res %d\n", (int) state);
        return -1;
    }

    if (state_isx031 != oc::ARGUS_STATE::OK) {
        printf("res %d\n", (int) state_isx031);
        return -1;
    }

    cv::Mat rgb = cv::Mat(config.mHeight, config.mWidth, CV_8UC4, 1);
    cv::Mat rgb_isx031 = cv::Mat(config_1.mHeight, config_1.mWidth, CV_8UC4, 1);

    cv::Mat rgb_d, rgb2_d;

    std::cout << "Press 's' to save images" << std::endl;

    char key = ' ';
    int image_count = 0;
    while (key != 'q') {
        if (camera_0.isNewFrame() && camera_1.isNewFrame()) {
            memcpy(rgb.data, camera_0.getPixels(), camera_0.getWidth() * camera_0.getHeight() * camera_0.getNumberOfChannels());
            memcpy(rgb_isx031.data, camera_1.getPixels(), camera_1.getWidth() * camera_1.getHeight() * camera_1.getNumberOfChannels());
            //std::cout<<" TS : "<<camera_0.getImageTimestampinUs()<<std::endl;


            ///Only with argus ///
            //std::cout<<" Exposure Time : "<<camera.getFrameExposureTime()<<" us"<<std::endl;
            //std::cout<<" Analog Gain : "<<camera.getAnalogFrameGain()<<" dB"<<std::endl;
            //std::cout<<" Digital Gain : "<<camera.getDigitalFrameGain()<<std::endl;
            ///////////////////

            cv::resize(rgb, rgb_d, cv::Size(1280, 720));
            cv::resize(rgb_isx031, rgb2_d, cv::Size(1280, 720));

            cv::imshow("image Left", rgb_d);
            cv::imshow("image Right", rgb2_d);
            key = cv::waitKey(2);

            if (key == 's') {
                // saves the images
                cv::imwrite("image_left_" + std::to_string(image_count) + ".png", rgb);
                cv::imwrite("image_right_" + std::to_string(image_count) + ".png", rgb_isx031);
                std::cout << "images created." << std::endl;
                image_count++;
            }

        } else
            usleep(100);


    }
    camera_0.closeCamera();
    camera_1.closeCamera();

    return 0;

}

