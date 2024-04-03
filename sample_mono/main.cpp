#include <memory>
#include <time.h>
#include <unistd.h>
#include <atomic>
#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"

int main(int argc, char *argv[]) {

    int camera_id_0 = 0;
    if (argc > 1) camera_id_0 = atoi(argv[1]);
    

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

    oc::ArgusCameraConfig config;

    if(devs.size() >= 1) {
        if(devs.at(camera_id_0).badge == "zedx_imx678") {
            std::cout << "ZED One 4K detected!" << std::endl;
            config.mFPS = 15;
            config.mWidth = 3856;
            config.mHeight = 2180; 
        } else if(devs.at(camera_id_0).badge == "zedx_ar0234") {
            std::cout << "ZED One GS detected!" << std::endl;
            config.mFPS = 30;
            config.mWidth = 1920;
            config.mHeight = 1200; 
        }
    } else {
        std::cerr << "No cameras detected" << std::endl;
        return -1;
    }

    config.mDeviceId = (camera_id_0);
    config.verbose_level = 3;

    oc::ArgusBayerCapture camera_0;
    oc::ARGUS_STATE state_cam0 = camera_0.openCamera(config);
    if (state_cam0 != oc::ARGUS_STATE::OK) {
        std::cerr << "Failed to open Camera, error code " << ARGUS_STATE2str(state_cam0) << std::endl;
        return -1;
    }

    cv::Mat rgb_d, rgb_cam0;
    rgb_cam0 = cv::Mat(config.mHeight, config.mWidth, CV_8UC4, 1);

    std::cout << "Press 's' to save images" << std::endl;

    char key = ' ';
    int image_count = 0;
    while (key != 'q') {
        if (camera_0.isNewFrame()) {
            memcpy(rgb_cam0.data, camera_0.getPixels(), camera_0.getWidth() * camera_0.getHeight() * camera_0.getNumberOfChannels());
            std::cout<<" TS : "<< camera_0.getImageTimestampinUs() <<std::endl;


            ///Only with argus ///
            std::cout<<" Exposure Time : "<<camera_0.getFrameExposureTime()<<" us"<<std::endl;
            std::cout<<" Analog Gain : "<<camera_0.getAnalogFrameGain()<<" dB"<<std::endl;
            std::cout<<" Digital Gain : "<<camera_0.getDigitalFrameGain()<<std::endl;
            ///////////////////

            cv::resize(rgb_cam0, rgb_d, cv::Size(1280, 720));

            cv::imshow("image", rgb_d);
            key = cv::waitKey(2);

            if (key == 's') {
                // saves the images
                cv::imwrite("image_left_" + std::to_string(image_count) + ".png", rgb_cam0);
                std::cout << "images created." << std::endl;
                image_count++;
            }
        } else
            usleep(100);


    }
    camera_0.closeCamera();

    return 0;

}

