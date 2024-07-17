#include <memory>
#include <time.h>
#include <fstream>
#include <unistd.h>
#include <atomic>
#include "ArgusCapture.hpp"
#include "opencv2/opencv.hpp"
#include <filesystem>
#include <deque>


#define SAFE_DELETE(e) if (e) { \
                        delete(e);\
                        e = NULL;}\

// CHANGE THIS PARAM BASED ON THE CHECKERBOARD USED
// https://docs.opencv.org/4.x/da/d0d/tutorial_camera_calibration_pattern.html
int target_w = 9; // number of horizontal inner edges
int target_h = 6; // number of vertical inner edges
float square_size = 25.0; // mm

std::string folder = "/tmp/zed-one/image/";
std::string output_filename = "SN_ZEDONES.conf";
int MIN_IMAGE = 5;
int verbose = 0;

namespace fs = std::filesystem;

struct extrinsic_checker {
    float rot_x_min;
    float rot_y_min;
    float rot_z_min;
    float rot_x_max;
    float rot_y_max;
    float rot_z_max;

    float rot_x_delta;
    float rot_y_delta;
    float rot_z_delta;

    float d_min;
    float d_max;
    float distance_tot;
};

struct SyncMat{
    cv::Mat rgb_img;
    uint64 ts; // in micro second
};



std::deque<SyncMat*> imageCaptureQueue[2];
std::mutex mutex_internal[2];



void ingestImageInQueue(oc::ArgusBayerCapture &camera,int side)
{
    while(camera.isOpened())
    {
        if(camera.isNewFrame())
        {
            std::lock_guard<std::mutex> guard(mutex_internal[side]);
            cv::Mat rgb = cv::Mat(camera.getHeight(), camera.getWidth(), CV_8UC4, 1);

            memcpy(rgb.data, camera.getPixels(), camera.getWidth() * camera.getHeight() * camera.getNumberOfChannels());
            SyncMat* img = new SyncMat();
            rgb.copyTo(img->rgb_img);
            img->ts = camera.getImageTimestampinUs();
            imageCaptureQueue[side].push_back(img);

            if (imageCaptureQueue[side].size()>5)
            {
                SyncMat* tmp  = imageCaptureQueue[side].front();
                SAFE_DELETE(tmp)
                imageCaptureQueue[side].pop_front();
            }
        }
    }
}

int SyncCameraPair(cv::Mat &rgb_l,cv::Mat &rgb_r);


std::map<std::string, std::string> parseArguments(int argc, char* argv[]);
bool writeTextCenter(cv::Mat& image, float rot_x, float rot_y, float rot_z, float distance, int fontSize);
bool WriteConfFile(cv::Mat& intrinsic_left, cv::Mat& intrinsic_right, cv::Mat& distortion_left, cv::Mat& distortion_right, cv::Mat& translation, cv::Mat& rotation, int model);
bool CheckBucket(int min_h, int max_h, int min_w, int max_w, bool min, std::vector<std::vector<cv::Point2f>> pts);
float CheckCoverage(std::vector<std::vector<cv::Point2f>> pts, cv::Size imgSize);
int TryCalibration(std::string folder, int target_w, int target_h, float square_size);
void checkRT(extrinsic_checker& checker_, cv::Mat r, cv::Mat t);

/// Calibration condition
///

const float min_coverage = 10; // in percentage
const float min_rotation = 60; // in degrees
const float acceptable_rotation = 50; // in degrees
const float min_distance = 200; // in mm
const float acceptable_distance = 150; // in mm

std::vector<std::vector<cv::Point2f>> pts_detected;

std::vector<cv::Point2f> square_valid;
int bucketsize = 480;
const int MinPts = 10;
const int MaxPts = 90;
cv::Scalar info_color = cv::Scalar(50,205,50);

int main(int argc, char *argv[]) {

    int camera_id_0 = 0;
    int camera_id_1 = 1;
    if(argc > 1) {
        camera_id_0 = atoi(argv[1]);
        camera_id_1 = atoi(argv[2]);
    }

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

    config.mDeviceId = camera_id_0;
    config.verbose_level = 3;

    oc::ArgusCameraConfig config_1;
    config_1.mDeviceId = camera_id_1;
    config_1.mFPS = config.mFPS;
    config_1.mWidth = config.mWidth;
    config_1.mHeight = config.mHeight;
    config_1.verbose_level = config.verbose_level;

    oc::ArgusBayerCapture camera_0, camera_1;
    oc::ARGUS_STATE state = camera_0.openCamera(config);
    oc::ARGUS_STATE state_isx031 = camera_1.openCamera(config_1);

    if (state != oc::ARGUS_STATE::OK) {
        std::cerr << "Failed to open Camera, error code " << ARGUS_STATE2str(state) << std::endl;
        return -1;
    }

    if (state_isx031 != oc::ARGUS_STATE::OK) {
        std::cerr << "Failed to open Camera, error code " << ARGUS_STATE2str(state) << std::endl;
        return -1;
    }


    std::cout << "The calibration requires a checkerboard of known characteristics" << std::endl;
    std::cout << "Expected checkerboard square size are " << target_w << "x" << target_h << " " << square_size << "mm" << std::endl;
    std::cout << "Change those values in the code depending on your checkerboard !" << std::endl;

    cv::Mat rgb_d, rgb2_d, rgb_d_fill;
    bool start_calibration = false;

    extrinsic_checker checker;
    float cov_left = 100;
    bool angle_clb = false;
    cv::Mat K_left, D_left, r_left, t_left;
    std::vector<cv::Point3f> pts_obj_;
    for (int i = 0; i < target_h; i++) {
        for (int j = 0; j < target_w; j++) {
            pts_obj_.push_back(cv::Point3f(square_size*j, square_size*i, 0));
        }
    }

    std::vector<std::vector < cv::Point3f>> pts_obj_tot;

    // Check if the folder exists
    if (!fs::exists(folder)) if(!fs::create_directories(folder)) std::cerr << "Error creating folder!\n";
    

    auto display_size = cv::Size(1280*0.75, 720*0.75);

    char key = ' ';
    int image_count = 0;
    bool coverage_mode = false;
    bool calibration_done = false;
    bool very_first_image=true;
    bool missing_left_target_on_last_pic = false;
    bool missing_right_target_on_last_pic = false;

    cv::Mat rgb_l;
    cv::Mat rgb_r;

    //Start image grabber Left and Right
    std::thread runner_left(ingestImageInQueue,std::ref(camera_0),0);
    std::thread runner_right(ingestImageInQueue,std::ref(camera_1),1);


    // Number of area to fill 4 horizontally
    bucketsize = camera_0.getWidth()/4;

    while (key != 'q') {
        if (SyncCameraPair(rgb_l,rgb_r)==0) {

            if(rgb_l.area()==0 || rgb_r.area()==0)
                continue;

            cv::resize(rgb_l, rgb_d, display_size);
            cv::resize(rgb_r, rgb2_d, display_size);

            if (!angle_clb && !calibration_done) {
                cv::Mat rgb_with_lack_of_pts;
                std::vector<cv::Mat> channels;
                cv::split(rgb_l, channels);
                cv::Mat blank = cv::Mat::zeros(cv::Size(camera_0.getWidth(), camera_0.getHeight()), CV_8UC1);
                float x_end,y_end;
                float x_max = 0;
                for (int i = 0; i < square_valid.size(); i++) {
                    if(square_valid.at(i).x + bucketsize > blank.size[1])
                        x_end = blank.size[1];
                    else
                        x_end = square_valid.at(i).x + bucketsize;
                    if(square_valid.at(i).y + bucketsize > blank.size[0])
                        y_end = blank.size[0];
                    else
                        y_end = square_valid.at(i).y + bucketsize;
                    if(square_valid.at(i).x>x_max)
                        x_max = square_valid.at(i).x;
                    cv::rectangle(blank, square_valid.at(i), cv::Point(x_end, y_end), cv::Scalar(128, 0, 128), -1);
                }
                channels[0] = channels[0] - blank;
                channels[2] = channels[2] - blank;
                cv::merge(channels, rgb_with_lack_of_pts);
                cv::resize(rgb_with_lack_of_pts, rgb_d_fill, display_size);
            } else {
                cv::resize(rgb_l, rgb_d_fill, display_size);
            }

            cv::Mat display, display_info;
            cv::hconcat(rgb_d_fill, rgb2_d, display);
            cv::Mat text_info = cv::Mat::ones(cv::Size(display.size[1], 200), display.type());

            if (angle_clb) {
                start_calibration = writeTextCenter(text_info, checker.rot_x_delta, checker.rot_y_delta, checker.rot_z_delta, checker.distance_tot, 1);
                coverage_mode = false;
            }

            cv::vconcat(display, text_info, display_info);

            if(!calibration_done) {
                std::stringstream ss;
                ss << std::fixed << std::setprecision(2) << square_size;
                if(very_first_image) {
                    cv::putText(display_info, std::string("Expected checkerboard " + std::to_string(target_w) + "x" + std::to_string(target_h) + " " + ss.str() +"mm").c_str(), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200,0,0), 2);
                    cv::putText(display_info, std::string("Make sure the Left and Right images are not inverted!").c_str(), cv::Point(10, 40), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200,0,0), 2);
                }
                if(missing_right_target_on_last_pic)
                    cv::putText(display_info, "Missing target on image right", cv::Point(10, display.size[0]+105), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,200), 2);
                if(missing_left_target_on_last_pic)
                    cv::putText(display_info, "Missing target on image left", cv::Point(10, display.size[0]+80), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,0,200), 2);

                cv::putText(display_info, "Press 's' to save an image", cv::Point(10, display.size[0]+25), cv::FONT_HERSHEY_SIMPLEX, 0.8, info_color, 2);
                if(coverage_mode)
                    cv::putText(display_info, "Keep going until the green covers the image, it represents coverage", cv::Point(10, display.size[0]+55), cv::FONT_HERSHEY_SIMPLEX, 0.6, info_color, 1);
            } else {
                cv::putText(display_info, "Calibration done! File written in " + output_filename, cv::Point(10, display.size[0]+25), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0,255,0), 2);
                cv::putText(display_info, "Press 'q' to exit", cv::Point(10, display.size[0]+75), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0,255,0), 2);
            }


            cv::imshow("image ", display_info);
            key = cv::waitKey(2);

            if (!calibration_done && key == 's') {
                if (!angle_clb) coverage_mode = true;
                std::vector<cv::Point2f> pts_l,pts_r;
                bool find_l = cv::findChessboardCorners(rgb_l, cv::Size(target_w, target_h), pts_l, 3);
                bool find_r = cv::findChessboardCorners(rgb_r, cv::Size(target_w, target_h), pts_r, 3);

                if(!find_r)
                    missing_right_target_on_last_pic = true;
                else
                    missing_right_target_on_last_pic = false;
                if(!find_l)
                    missing_left_target_on_last_pic = true;
                else
                    missing_left_target_on_last_pic = false;


                if (find_l && find_r) {
                    very_first_image = false;
                    if (!angle_clb) {
                        pts_detected.push_back(pts_l);
                        cov_left = CheckCoverage(pts_detected, cv::Size(camera_0.getWidth(), camera_0.getHeight()));
                        pts_obj_tot.push_back(pts_obj_);
                        std::cout << "coverage : " << (1-cov_left)*100 << "%" << std::endl;
                        if (cov_left < 0.1) {
                            cv::Mat rvec(1, 3, CV_64FC1);
                            cv::Mat tvec(1, 3, CV_64FC1);
                            float err = cv::calibrateCamera(pts_obj_tot, pts_detected, cv::Size(camera_0.getWidth(), camera_0.getHeight()), K_left, D_left, r_left, t_left);
                            bool find_ = cv::solvePnP(pts_obj_, pts_l, K_left, D_left, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

                            checker.rot_x_min = rvec.at<double>(0) * 180 / M_PI;
                            checker.rot_x_max = rvec.at<double>(0) * 180 / M_PI;
                            checker.rot_y_min = rvec.at<double>(1) * 180 / M_PI;
                            checker.rot_y_max = rvec.at<double>(1) * 180 / M_PI;
                            checker.rot_z_min = rvec.at<double>(2) * 180 / M_PI;
                            checker.rot_z_max = rvec.at<double>(2) * 180 / M_PI;

                            checker.d_min = sqrt(pow(tvec.at<double>(0), 2) + pow(tvec.at<double>(1), 2) + pow(tvec.at<double>(2), 2));
                            checker.d_max = sqrt(pow(tvec.at<double>(0), 2) + pow(tvec.at<double>(1), 2) + pow(tvec.at<double>(2), 2));

                            angle_clb = true;
                        }
                    } else {
                        cv::Mat rvec(1, 3, CV_64FC1);
                        cv::Mat tvec(1, 3, CV_64FC1);
                        pts_detected.push_back(pts_l);
                        bool find_ = cv::solvePnP(pts_obj_, pts_l, K_left, D_left, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
                        if (find_) {
                            checkRT(checker, rvec, tvec);
                        }
                    }

                }

                // saves the images
                cv::imwrite(folder + "image_left_" + std::to_string(image_count) + ".png", rgb_l);
                cv::imwrite(folder + "image_right_" + std::to_string(image_count) + ".png", rgb_r);
                std::cout << "images created." << std::endl;
                image_count++;

            } else if (start_calibration) {
                int err = TryCalibration(folder, target_w, target_h, square_size);
                if (err == 0) {
                    calibration_done = true;
                    start_calibration = false;
                    angle_clb = false;
                    std::cout << "CALIBRATION success" << std::endl;
                } else {
                    std::cout << "CALIBRATION fail" << std::endl;
                }
            }

        } else
            usleep(100);



    }
    camera_0.closeCamera();
    camera_1.closeCamera();


    runner_left.join();
    runner_right.join();


    return 0;

}

// Function to perform linear interpolation
inline float interpolate(float x, float x0, float x1, float y0=0, float y1=100) {
     float interpolatedValue = y0 + (x - x0) * (y1 - y0) / (x1 - x0);
    interpolatedValue = (interpolatedValue < y0) ? y0 : ((interpolatedValue > y1) ? y1 : interpolatedValue); // clamp
    return interpolatedValue;
}

bool writeTextCenter(cv::Mat& image, float rot_x, float rot_y, float rot_z, float distance,  int fontSize) {
    bool status = false;
    // Define text from rotation and distance

    // Convert float values to string with two decimal places
    std::stringstream ss_rot_x, ss_rot_y, ss_rot_z, ss_distance;


    int rot_x_idx = interpolate(rot_x, 0, min_rotation);
    int rot_y_idx = interpolate(rot_y, 0, min_rotation);
    int rot_z_idx = interpolate(rot_z, 0, min_rotation);
    int distance_idx = interpolate(distance, 0, min_distance);

    ss_rot_x << "Rotation x : " << std::fixed << std::setprecision(1) << rot_x_idx << "%   ";
    ss_rot_y << "Rotation y : " << std::fixed << std::setprecision(1) << rot_y_idx << "%   ";
    ss_rot_z << "Rotation z : " << std::fixed << std::setprecision(1) << rot_z_idx << "%   ";
    ss_distance << "Distance : " << std::fixed << std::setprecision(1) << distance_idx << "%";


    std::string text1 = ss_rot_x.str();
    std::string text2 = ss_rot_y.str();
    std::string text3 = ss_rot_z.str();
    std::string text4 = ss_distance.str();

    std::string text = text1 + text2 + text3 + text4;

    cv::Scalar color1, color2, color3, color4;

    // Condition on colors
    if (rot_x > min_rotation)
        color1 = cv::Scalar(0, 255, 0);
    else if (rot_x >= acceptable_rotation)
        color1 = cv::Scalar(0, 128, 255);
    else
        color1 = cv::Scalar(0, 0, 255);

    if (rot_y > min_rotation)
        color2 = cv::Scalar(0, 255, 0);
    else if (rot_y >= acceptable_rotation)
        color2 = cv::Scalar(0, 128, 255);
    else
        color2 = cv::Scalar(0, 0, 255);

    if (rot_z > min_rotation)
        color3 = cv::Scalar(0, 255, 0);
    else if (rot_z >= acceptable_rotation)
        color3 = cv::Scalar(0, 128, 255);
    else
        color3 = cv::Scalar(0, 0, 255);

    if (distance > min_distance)
        color4 = cv::Scalar(0, 255, 0);
    else if (distance >= acceptable_distance)
        color4 = cv::Scalar(0, 128, 255);
    else
        color4 = cv::Scalar(0, 0, 255);

    // Get image dimensions
    int width = image.cols;
    int height = image.rows;

    // Calculate text size
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, fontSize, 1, &baseline);

    // Calculate text position
    int x = (width - textSize.width) / 2;
    int y = (height + textSize.height) / 2;

    status = (rot_x > min_rotation) && (rot_y > min_rotation) && (rot_z > min_rotation) && (distance > min_distance);

    // Draw text on image with different colors
    cv::putText(image, text1, cv::Point(x, y), cv::FONT_HERSHEY_SIMPLEX, fontSize, color1, 2);
    cv::putText(image, text2, cv::Point(x + textSize.width / 4, y), cv::FONT_HERSHEY_SIMPLEX, fontSize, color2, 2);
    cv::putText(image, text3, cv::Point(x + textSize.width / 2, y), cv::FONT_HERSHEY_SIMPLEX, fontSize, color3, 2);
    cv::putText(image, text4, cv::Point(x + 3 * textSize.width / 4, y), cv::FONT_HERSHEY_SIMPLEX, fontSize, color4, 2);

    if(status)
        cv::putText(image, "Starting calibration...", cv::Point(10, y-40), cv::FONT_HERSHEY_SIMPLEX, 0.8, info_color, 2);

    return status;
}

int TryCalibration(std::string folder, int target_w, int target_h, float square_size) {

    std::vector<cv::Mat> left_images, right_images;

    /// Read images

    cv::Size imageSize = cv::Size(0, 0);
    int img_number = 0;

    std::cout << folder + "image_left_" + std::to_string(img_number) + ".png" << std::endl;

    while (!cv::imread(folder + "image_left_" + std::to_string(img_number) + ".png").empty()) {
        std::cout << "Reading img in folder : " + folder + "image_left_" + std::to_string(img_number) + ".png" << std::endl;
        cv::Mat left_image = cv::imread(folder + "image_left_" + std::to_string(img_number) + ".png");
        cv::Mat right_image = cv::imread(folder + "image_right_" + std::to_string(img_number) + ".png");
        if (!left_image.empty() && !right_image.empty()) {
            if (imageSize.width == 0)
                imageSize = cv::Size(left_image.size[1], left_image.size[0]);
            else
                if (imageSize != cv::Size(left_image.size[1], left_image.size[0])) {
                std::cout << "Image number " << img_number << " does not have the same size as the previous ones" << std::endl;
                break;
            }

            cv::Mat grey_l, grey_r;

            cv::cvtColor(left_image, grey_l, cv::COLOR_BGRA2GRAY);
            cv::cvtColor(right_image, grey_r, cv::COLOR_BGRA2GRAY);

            left_images.push_back(grey_l);
            right_images.push_back(grey_r);
        }
        img_number++;
    }

    std::cout << img_number << " images opened" << std::endl;

    /// Point detection
    ///
    ///

    // Define object points of the target
    std::vector<std::vector < cv::Point3f>> object_points;
    std::vector<cv::Point3f> object_points_;
    for (int i = 0; i < target_h; i++) {
        for (int j = 0; j < target_w; j++) {
            object_points_.push_back(cv::Point3f(square_size*j, square_size*i, 0));
        }
    }

    std::vector<std::vector < cv::Point2f>> pts_l, pts_r;

    for (int i = 0; i < left_images.size(); i++) {
        std::vector<cv::Point2f> pts_l_, pts_r_;
        bool find_l = cv::findChessboardCorners(left_images.at(i), cv::Size(target_w, target_h), pts_l_, 3);
        bool find_r = cv::findChessboardCorners(right_images.at(i), cv::Size(target_w, target_h), pts_r_, 3);

        if (find_l && find_r) {
            cv::cornerSubPix(left_images.at(i), pts_l_, cv::Size(5, 5), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            cv::cornerSubPix(right_images.at(i), pts_r_, cv::Size(5, 5), cv::Size(-1, -1),
                    cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001));

            pts_l.push_back(pts_l_);
            pts_r.push_back(pts_r_);
            object_points.push_back(object_points_);
            std::cout << "Target detected on image " << i << std::endl;
        } else {
            std::cout << "No target detected on image " << i << std::endl;
        }
    }

    /// Compute calibration

    //Check coverage
    float cov_left = CheckCoverage(pts_l, imageSize);
    float cov_right = CheckCoverage(pts_r, imageSize);

    std::cout << "coverage left : " << (1-cov_left) * 100 << "%" << std::endl;
    std::cout << "coverage right : " << (1-cov_right) * 100 << "%"  << std::endl;

    if (pts_l.size() < MIN_IMAGE)
        std::cout << "Not enough images with the target detected" << std::endl;
    else {
        std::cout << "enough points detected" << std::endl;
        cv::Mat intrinsic_l, intrinsic_r, distortion_l, distortion_r;
        cv::Mat R_, T, r_, E_, F_;

        int flag = cv::CALIB_ZERO_DISPARITY;//cv::CALIB_RATIONAL_MODEL;
        float err = cv::stereoCalibrate(object_points, pts_l, pts_r, intrinsic_l, distortion_l, intrinsic_r, distortion_r, imageSize,
                R_, T, E_, F_, flag, cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 1000, 0.001));

        cv::Rodrigues(R_, r_);

        std::cout << "Reprojection error : " << err << std::endl;
        std::cout << "intrinsic mat left : " << intrinsic_l << std::endl;
        std::cout << "\ndistortion mat right : " << intrinsic_r << std::endl;
        std::cout << "\nintrinsic mat left : " << distortion_l << std::endl;
        std::cout << "\ndistortion mat right : " << distortion_r << std::endl;
        std::cout << "\nextrinsic : \n\n translation : " << T << "\n\nRotation : " << r_ << std::endl;

        /// Compute rectified field of view horizontal

        cv::Mat R1, R2, P1, P2, matQ;
        cv::stereoRectify(intrinsic_l, distortion_l, intrinsic_r, distortion_r, imageSize, r_, T, R1, R2, P1, P2, matQ, cv::CALIB_ZERO_DISPARITY, 0, imageSize);
        cv::Mat mxl, myl, mxr, myr;
        cv::initUndistortRectifyMap(intrinsic_l, distortion_l, R1, P1, imageSize, CV_32FC1, mxl, myl);
        cv::initUndistortRectifyMap(intrinsic_r, distortion_r, R2, P2, imageSize, CV_32FC1, mxr, myr);

        std::cout << matQ.at<double>(3, 2) << std::endl;

        double FOV_x = 2.0 * atan(static_cast<double> (imageSize.width) / (2.0 * matQ.at<double>(2, 3))) * 180.0 / CV_PI;

        std::cout << "Rectified field of view : " << FOV_x << std::endl;

        int model = 0;
        if (imageSize.height == 1200)
            model = 1;
        else if (imageSize.height == 2160)
            model = 2;

        WriteConfFile(intrinsic_l, intrinsic_r, distortion_l, distortion_r, T, r_, model);
    }

    return EXIT_SUCCESS;
}

bool WriteConfFile(cv::Mat& intrinsic_left, cv::Mat& intrinsic_right, cv::Mat& distortion_left, cv::Mat& distortion_right, cv::Mat& translation, cv::Mat& rotation, int model) {
    // Write parameters to a text file
    std::ofstream outfile(output_filename);
    if (!outfile.is_open()) {
        std::cerr << "Unable to open output file." << std::endl;
        return 1;
    }

    if (model == 1) //  AR0234
    {
        std::cout << "intr left : " << intrinsic_left << std::endl;
        outfile << "[LEFT_CAM_FHD1200]\n";
        outfile << "fx = " << intrinsic_left.at<double>(0, 0) << "\n";
        outfile << "fy = " << intrinsic_left.at<double>(1, 1) << "\n";
        outfile << "cx = " << intrinsic_left.at<double>(0, 2) << "\n";
        outfile << "cy = " << intrinsic_left.at<double>(1, 2) << "\n\n";

        outfile << "[RIGHT_CAM_FHD1200]\n";
        outfile << "fx = " << intrinsic_right.at<double>(0, 0) << "\n";
        outfile << "fy = " << intrinsic_right.at<double>(1, 1) << "\n";
        outfile << "cx = " << intrinsic_right.at<double>(0, 2) << "\n";
        outfile << "cy = " << intrinsic_right.at<double>(1, 2) << "\n\n";

        outfile << "[LEFT_CAM_FHD]\n";
        outfile << "fx = " << intrinsic_left.at<double>(0, 0) << "\n";
        outfile << "fy = " << intrinsic_left.at<double>(1, 1) << "\n";
        outfile << "cx = " << intrinsic_left.at<double>(0, 2) << "\n";
        outfile << "cy = " << intrinsic_left.at<double>(1, 2) - 60 << "\n\n";

        outfile << "[RIGHT_CAM_FHD]\n";
        outfile << "fx = " << intrinsic_right.at<double>(0, 0) << "\n";
        outfile << "fy = " << intrinsic_right.at<double>(1, 1) << "\n";
        outfile << "cx = " << intrinsic_right.at<double>(0, 2) << "\n";
        outfile << "cy = " << intrinsic_right.at<double>(1, 2) - 60 << "\n\n";

        outfile << "[LEFT_CAM_SVGA]\n";
        outfile << "fx = " << intrinsic_left.at<double>(0, 0) / 2 << "\n";
        outfile << "fy = " << intrinsic_left.at<double>(1, 1) / 2 << "\n";
        outfile << "cx = " << intrinsic_left.at<double>(0, 2) / 2 << "\n";
        outfile << "cy = " << intrinsic_left.at<double>(1, 2) / 2 << "\n\n";

        outfile << "[RIGHT_CAM_SVGA]\n";
        outfile << "fx = " << intrinsic_right.at<double>(0, 0) / 2 << "\n";
        outfile << "fy = " << intrinsic_right.at<double>(1, 1) / 2 << "\n";
        outfile << "cx = " << intrinsic_right.at<double>(0, 2) / 2 << "\n";
        outfile << "cy = " << intrinsic_right.at<double>(1, 2) / 2 << "\n\n";

        // Add other parameters for other cameras...

        outfile << "[LEFT_DISTO]\n";
        outfile << "k1 = " << distortion_left.at<double>(0) << "\n";
        outfile << "k2 = " << distortion_left.at<double>(1) << "\n";
        outfile << "p1 = " << distortion_left.at<double>(2) << "\n";
        outfile << "p2 = " << distortion_left.at<double>(3) << "\n";
        outfile << "k3 = " << distortion_left.at<double>(4) << "\n";
        outfile << "k4 = " << distortion_left.at<double>(5) << "\n";
        outfile << "k5 = " << distortion_left.at<double>(6) << "\n";
        outfile << "k6 = " << distortion_left.at<double>(7) << "\n\n";


        outfile << "[RIGHT_DISTO]\n";
        outfile << "k1 = " << distortion_right.at<double>(0) << "\n";
        outfile << "k2 = " << distortion_right.at<double>(1) << "\n";
        outfile << "p1 = " << distortion_right.at<double>(2) << "\n";
        outfile << "p2 = " << distortion_right.at<double>(3) << "\n";
        outfile << "k3 = " << distortion_right.at<double>(4) << "\n";
        outfile << "k4 = " << distortion_right.at<double>(5) << "\n";
        outfile << "k5 = " << distortion_right.at<double>(6) << "\n";
        outfile << "k6 = " << distortion_right.at<double>(7) << "\n\n";

        outfile << "[STEREO]\n";
        outfile << "Baseline = " << -translation.at<double>(0) << "\n";
        outfile << "TY = " << translation.at<double>(1) << "\n";
        outfile << "TZ = " << translation.at<double>(2) << "\n";
        outfile << "CV_FHD = " << rotation.at<double>(1) << "\n";
        outfile << "CV_SVGA = " << rotation.at<double>(1) << "\n";
        outfile << "CV_FHD1200 = " << rotation.at<double>(1) << "\n";
        outfile << "RX_FHD = " << rotation.at<double>(0) << "\n";
        outfile << "RX_SVGA = " << rotation.at<double>(0) << "\n";
        outfile << "RX_FHD1200 = " << rotation.at<double>(0) << "\n";
        outfile << "RZ_FHD = " << rotation.at<double>(2) << "\n";
        outfile << "RZ_SVGA = " << rotation.at<double>(2) << "\n";
        outfile << "RZ_FHD1200 = " << rotation.at<double>(2) << "\n\n";

        // Add other parameters for other stereo parameters if needed...

        outfile << "[MISC]\n";
        outfile << "Sensor_ID = 1\n\n";

        outfile.close();
        std::cout << "Parameter file written successfully." << std::endl;
        return true;
    } else if (model == 2) //  IMX678
    {
        outfile << "[LEFT_CAM_4k]\n";
        outfile << "fx = " << intrinsic_left.at<double>(0, 0) << "\n";
        outfile << "fy = " << intrinsic_left.at<double>(1, 1) << "\n";
        outfile << "cx = " << intrinsic_left.at<double>(0, 2) << "\n";
        outfile << "cy = " << intrinsic_left.at<double>(1, 2) << "\n\n";

        outfile << "[RIGHT_CAM_4k]\n";
        outfile << "fx = " << intrinsic_right.at<double>(0, 0) << "\n";
        outfile << "fy = " << intrinsic_right.at<double>(1, 1) << "\n";
        outfile << "cx = " << intrinsic_right.at<double>(0, 2) << "\n";
        outfile << "cy = " << intrinsic_right.at<double>(1, 2) << "\n\n";

        outfile << "[LEFT_CAM_FHD]\n";
        outfile << "fx = " << intrinsic_left.at<double>(0, 0) / 2 << "\n";
        outfile << "fy = " << intrinsic_left.at<double>(1, 1) / 2 << "\n";
        outfile << "cx = " << intrinsic_left.at<double>(0, 2) / 2 << "\n";
        outfile << "cy = " << intrinsic_left.at<double>(1, 2) / 2 << "\n\n";

        outfile << "[RIGHT_CAM_FHD]\n";
        outfile << "fx = " << intrinsic_right.at<double>(0, 0) / 2 << "\n";
        outfile << "fy = " << intrinsic_right.at<double>(1, 1) / 2 << "\n";
        outfile << "cx = " << intrinsic_right.at<double>(0, 2) / 2 << "\n";
        outfile << "cy = " << intrinsic_right.at<double>(1, 2) / 2 << "\n\n";

        outfile << "[LEFT_CAM_FHD1200]\n";
        outfile << "fx = " << intrinsic_left.at<double>(0, 0) << "\n";
        outfile << "fy = " << intrinsic_left.at<double>(1, 1) << "\n";
        outfile << "cx = " << intrinsic_left.at<double>(0, 2)-(3840 - 1920) / 2 << "\n";
        outfile << "cy = " << intrinsic_left.at<double>(1, 2)-(2160 - 1200) / 2 << "\n\n";

        outfile << "[RIGHT_CAM_FHD1200]\n";
        outfile << "fx = " << intrinsic_right.at<double>(0, 0) << "\n";
        outfile << "fy = " << intrinsic_right.at<double>(1, 1) << "\n";
        outfile << "cx = " << intrinsic_right.at<double>(0, 2)-(3840 - 1920) / 2 << "\n";
        outfile << "cy = " << intrinsic_right.at<double>(1, 2)-(2160 - 1200) / 2 << "\n\n";

        // Add other parameters for other cameras...

        outfile << "[LEFT_DISTO]\n";
        outfile << "k1 = " << distortion_left.at<double>(0) << "\n";
        outfile << "k2 = " << distortion_left.at<double>(1) << "\n";
        outfile << "p1 = " << distortion_left.at<double>(2) << "\n";
        outfile << "p2 = " << distortion_left.at<double>(3) << "\n";
        outfile << "k3 = " << distortion_left.at<double>(4) << "\n";
        outfile << "k4 = " << distortion_left.at<double>(5) << "\n";
        outfile << "k5 = " << distortion_left.at<double>(6) << "\n";
        outfile << "k6 = " << distortion_left.at<double>(7) << "\n\n";


        outfile << "[RIGHT_DISTO]\n";
        outfile << "k1 = " << distortion_right.at<double>(0) << "\n";
        outfile << "k2 = " << distortion_right.at<double>(1) << "\n";
        outfile << "p1 = " << distortion_right.at<double>(2) << "\n";
        outfile << "p2 = " << distortion_right.at<double>(3) << "\n";
        outfile << "k3 = " << distortion_right.at<double>(4) << "\n";
        outfile << "k4 = " << distortion_right.at<double>(5) << "\n";
        outfile << "k5 = " << distortion_right.at<double>(6) << "\n";
        outfile << "k6 = " << distortion_right.at<double>(7) << "\n\n";

        outfile << "[STEREO]\n";
        outfile << "Baseline = " << -translation.at<double>(0) << "\n";
        outfile << "TY = " << translation.at<double>(1) << "\n";
        outfile << "TZ = " << translation.at<double>(2) << "\n";
        outfile << "CV_FHD = " << rotation.at<double>(1) << "\n";
        outfile << "CV_FHD1200 = " << rotation.at<double>(1) << "\n";
        outfile << "CV_4k = " << rotation.at<double>(1) << "\n";
        outfile << "RX_FHD = " << rotation.at<double>(0) << "\n";
        outfile << "RX_FHD1200 = " << rotation.at<double>(0) << "\n";
        outfile << "RX_4k = " << rotation.at<double>(0) << "\n";
        outfile << "RZ_FHD = " << rotation.at<double>(2) << "\n";
        outfile << "RZ_FHD1200 = " << rotation.at<double>(2) << "\n";
        outfile << "RZ_4k = " << rotation.at<double>(2) << "\n\n";

        // Add other parameters for other stereo parameters if needed...

        outfile << "[MISC]\n";
        outfile << "Sensor_ID = 2\n\n";

        outfile.close();
        std::cout << "Parameter file written successfully in " << output_filename << std::endl;
        return true;
    } else {
        std::cout << "The resolution for the calibration is not write\n\nUse 4k (3840x2160) for zedoneuhd and FHD1200 (1920x1200) for zedonegs" << std::endl;
        return false;
    }
}

bool CheckBucket(int min_h, int max_h, int min_w, int max_w, bool min, std::vector<std::vector<cv::Point2f>> pts) {
    int compteur = 0;

    for (int i = 0; i < pts.size(); i++) {
        for (int j = 0; j < pts.at(i).size(); j++) {
            if ((pts.at(i).at(j).x < max_w)&&(pts.at(i).at(j).x > min_w)) {
                if ((pts.at(i).at(j).y < max_h)&&(pts.at(i).at(j).y > min_h)) {
                    compteur = compteur + 1;
                }
            }
        }
    }

    if (min) {
        if (compteur > MinPts) return true;
        else return false;
    } else {
        if (compteur < MaxPts) return true;
        else return false;
    }

}

float CheckCoverage(std::vector<std::vector<cv::Point2f>> pts, cv::Size imgSize) {
    int min_h_ = 0;
    int min_w_ = 0;
    int max_h_ = bucketsize;
    int max_w_ = bucketsize;
    float tot = 0;
    float error = 0;

    while (min_h_ < imgSize.height) {
        if (max_h_ > imgSize.height)
            max_h_ = imgSize.height;
        max_w_ = bucketsize;
        min_w_ = 0;
        while (min_w_ < imgSize.width) {
            if (max_w_ > imgSize.width)
                max_w_ = imgSize.width;
            if (!CheckBucket(min_h_, max_h_, min_w_, max_w_, true, pts)) {
                error++;
            } else
                square_valid.push_back(cv::Point(min_w_, min_h_));
            min_w_ += bucketsize;
            max_w_ += bucketsize;
            tot++;

        }
        min_h_ += bucketsize;
        max_h_ += bucketsize;
    }
    return error / tot;
}

void checkRT(extrinsic_checker& checker_, cv::Mat r, cv::Mat tvec) {
    // check min
    if (checker_.rot_x_min > r.at<double>(0) *(180 / M_PI))
        checker_.rot_x_min = r.at<double>(0)*(180 / M_PI);
    if (checker_.rot_y_min > r.at<double>(1)*(180 / M_PI))
        checker_.rot_y_min = r.at<double>(1)*(180 / M_PI);
    if (checker_.rot_z_min > r.at<double>(2)*(180 / M_PI))
        checker_.rot_z_min = r.at<double>(2)*(180 / M_PI);

    //check max
    if (checker_.rot_x_max < r.at<double>(0)*(180 / M_PI))
        checker_.rot_x_max = r.at<double>(0)*(180 / M_PI);
    if (checker_.rot_y_max < r.at<double>(1)*(180 / M_PI))
        checker_.rot_y_max = r.at<double>(1)*(180 / M_PI);
    if (checker_.rot_z_max < r.at<double>(2)*(180 / M_PI))
        checker_.rot_z_max = r.at<double>(2)*(180 / M_PI);

    if (checker_.d_min > sqrt(pow(tvec.at<double>(0), 2) + pow(tvec.at<double>(1), 2) + pow(tvec.at<double>(2), 2)))
        checker_.d_min = sqrt(pow(tvec.at<double>(0), 2) + pow(tvec.at<double>(1), 2) + pow(tvec.at<double>(2), 2));
    if (checker_.d_max < sqrt(pow(tvec.at<double>(0), 2) + pow(tvec.at<double>(1), 2) + pow(tvec.at<double>(2), 2)))
        checker_.d_max = sqrt(pow(tvec.at<double>(0), 2) + pow(tvec.at<double>(1), 2) + pow(tvec.at<double>(2), 2));

    // compute deltas
    checker_.rot_x_delta = checker_.rot_x_max - checker_.rot_x_min;
    checker_.rot_y_delta = checker_.rot_y_max - checker_.rot_y_min;
    checker_.rot_z_delta = checker_.rot_z_max - checker_.rot_z_min;
    checker_.distance_tot = checker_.d_max - checker_.d_min;

    std::cout << "rot x : " << checker_.rot_x_delta << std::endl;
    std::cout << "rot y : " << checker_.rot_y_delta << std::endl;
    std::cout << "rot z : " << checker_.rot_z_delta << std::endl;
    std::cout << "dist : " << checker_.distance_tot << std::endl;

}



int SyncCameraPair(cv::Mat &rgb_l,cv::Mat &rgb_r)

{
  //  Get Queue size and return if empty (no new frame)
  int maxSizeLeftQueue = 0, maxSizeRightQueue = 0;
  {
    std::lock_guard<std::mutex> guard_l(mutex_internal[0]);
    maxSizeLeftQueue = imageCaptureQueue[0].size();
  }
  {
    std::lock_guard<std::mutex> guard_r(mutex_internal[1]);
    maxSizeRightQueue = imageCaptureQueue[1].size();
  }

  if (maxSizeLeftQueue<=0 || maxSizeRightQueue<=0)
    return 1;

  /////////////// Timestamp Synchronisation mechanism //////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////
  SyncMat* tmpImageTryLeft;
  SyncMat* tmpImageTryRight;
  bool found_in_right_queue = false;

  int iLTarget = 1;//maxSizeLeftQueue;
  int iRtarget = 1;//maxSizeRightQueue;
  // How this is done : //
  // Take last argusImage on the left queue (at the time it was lock).
  // Try to find the sync argusImage on the right queue started from the last image in queue. Then increase the counter if not found.
  // If no right image found, then increase the left counter in queue so that we take the last - 1 then retry.... until no image in queue.
  // If we found a right image that fits. break and output images.
  // At the end , free all the remaining queue since it won't be used (previous timestamp).
  do
    {
      {
        std::lock_guard<std::mutex> guard(mutex_internal[0]);
        if (maxSizeLeftQueue-iLTarget<0 || maxSizeLeftQueue-iLTarget>=imageCaptureQueue[0].size())
          return 1;

        tmpImageTryLeft = imageCaptureQueue[0].at(maxSizeLeftQueue-iLTarget);
      }
      uint64 targetTS = tmpImageTryLeft->ts;
      found_in_right_queue = false;
      uint64 rightTS = 0;

      do
        {
          {
            std::lock_guard<std::mutex> guard(mutex_internal[1]);
            if (maxSizeRightQueue-iRtarget<0 || maxSizeRightQueue-iRtarget>=imageCaptureQueue[1].size())
              return 1;

            tmpImageTryRight = imageCaptureQueue[1].at(maxSizeRightQueue-iRtarget);
          }
          rightTS = tmpImageTryRight->ts;
          if (abs((long long)rightTS-(long long)targetTS)<=2000) //SyncDiff must be lower than 2000us
            {
              found_in_right_queue = true;
              break;
            }
          iRtarget++;
          usleep(100);
        }
      while(maxSizeRightQueue-iRtarget>=0);


      if (found_in_right_queue)
        {
          {
          std::lock_guard<std::mutex> guardL(mutex_internal[0]);
          std::lock_guard<std::mutex> guardR(mutex_internal[1]);
          tmpImageTryLeft->rgb_img.copyTo(rgb_l);
          tmpImageTryRight->rgb_img.copyTo(rgb_r);
          }
          break;
        }
      iRtarget = 1;
      iLTarget++;
    }
  while(maxSizeLeftQueue-iLTarget>=0);


  //Not found... No new frame sync
  if (!found_in_right_queue)
    {
      return 1;
    }


  ///empy all the queue before it will not be used. //
  {
    std::lock_guard<std::mutex> guard_l(mutex_internal[0]);
    for (int p=0;p<=maxSizeLeftQueue-iLTarget;p++)
      {
        SyncMat* tmp  = imageCaptureQueue[0].front();
        SAFE_DELETE(tmp)
        imageCaptureQueue[0].pop_front();
      }
  }

  {
    std::lock_guard<std::mutex> guard_r(mutex_internal[1]);
    for (int p=0;p<=maxSizeRightQueue-iRtarget;p++)
      {
        SyncMat* tmp  = imageCaptureQueue[1].front();
        SAFE_DELETE(tmp)
        imageCaptureQueue[1].pop_front();

      }
  }
  return 0;

}
