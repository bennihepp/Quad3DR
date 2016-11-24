//==================================================
// video_streamer_bundlefusion_drone.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 7, 2016
//==================================================

#define DEBUG_IMAGE_COMPRESSION 0

#include <ait/BoostNetworkClientTCP.h>
#include <ait/BoostNetworkClientUDP.h>

#include <iostream>
#include <stdexcept>
#include <algorithm>
#include <functional>
#include <csignal>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <mutex>

#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include <ros/ros.h>

#include <librealsense/rs.hpp>

#include <ait/math.h>
#include <ait/video/video_source_zed.h>
#include <ait/video/StereoNetworkSensorManager.h>
#include <ait/video/EncodingGstreamerPipeline.h>
#include <ait/dji/RosDjiDrone.h>

const double LOCATION_TIMESTAMP_DIFF_THRESHOLD = 0.15;
// TODO
//std::chrono::seconds PIPELINE_PLAYING_TIMEOUT(5);
//unsigned int PIPELINE_TIMEOUT_MIN_NUM_FRAMES = 0;

volatile bool g_abort;

void signalHandler(int sig)
{
    std::cout << "Received CTRL-C. Aborting" << std::endl;
    g_abort = true;
    ros::shutdown();
}

class VideoSourceRealsense
{
public:
    VideoSourceRealsense(unsigned int width = 640, unsigned int height = 480)
        : width_(width), height_(height) {
        std::cout << "There are " << ctx_.get_device_count() << " connected RealSense devices." << std::endl;

        dev_ = ctx_.get_device(0);

        std::cout << "\nUsing device 0, an " << dev_->get_name() << std::endl;
        std::cout << "    Serial number: " << dev_->get_serial() << std::endl;
        std::cout << "    Firmware version: " << dev_->get_firmware_version() << std::endl;

        std::cout << "Enabling streams" << std::endl;
        dev_->enable_stream(rs::stream::depth, width, height_, rs::format::z16, 30);
        dev_->enable_stream(rs::stream::color, width_, height_, rs::format::rgb8, 30);
        dev_->enable_stream(rs::stream::infrared, width_, height_, rs::format::y8, 30);
        dev_->enable_stream(rs::stream::infrared2, width_, height_, rs::format::y8, 30);

        std::cout << "Getting depth scale" << std::endl;
        depth_scale_ = dev_->get_depth_scale();

        std::cout << "Setting options" << std::endl;
        setOptions();

        std::cout << "Starting" << std::endl;
        dev_->start();
    }

    template <typename T>
    void setOption(rs::option option, T value, const std::string& name) {
        try {
            std::cout << "Setting option " << name << " to value " << value << std::endl;
            dev_->set_option(option, value);
        } catch (const rs::error& err) {
                std::cout << "Failed to set option " << name << " to value " << value << std::endl;
            }
    }

    #define REALSENSE_SET_OPTION(name, value) setOption(rs::option:: name, value, #name)
    void setOptions() {
        REALSENSE_SET_OPTION(r200_emitter_enabled, 0);
        REALSENSE_SET_OPTION(r200_lr_gain, 400);
        REALSENSE_SET_OPTION(r200_lr_exposure, 164);
        REALSENSE_SET_OPTION(r200_lr_auto_exposure_enabled, 1);
        REALSENSE_SET_OPTION(r200_auto_exposure_mean_intensity_set_point, 512);
        REALSENSE_SET_OPTION(r200_auto_exposure_bright_ratio_set_point, 0);
        REALSENSE_SET_OPTION(r200_auto_exposure_kp_gain, 0);
        REALSENSE_SET_OPTION(r200_auto_exposure_kp_exposure, 0);
        REALSENSE_SET_OPTION(r200_auto_exposure_kp_dark_threshold, 0);
        REALSENSE_SET_OPTION(r200_auto_exposure_top_edge, 0);
        REALSENSE_SET_OPTION(r200_auto_exposure_bottom_edge, 0);
        REALSENSE_SET_OPTION(r200_auto_exposure_left_edge, 0);
        REALSENSE_SET_OPTION(r200_auto_exposure_right_edge, 0);
    }

    bool retrieveFrames(double* timestamp, cv::Mat& left_frame, cv::Mat& right_frame, cv::Mat& depth_frame) {
        dev_->wait_for_frames();

//        *timestamp = ros::Time::now().toSec();
        using clock = std::chrono::high_resolution_clock;
        *timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(clock::now().time_since_epoch()).count();

        const unsigned int width = width_;
        const unsigned int height = height_;

        const uint8_t* left_data = reinterpret_cast<const uint8_t*>(dev_->get_frame_data(rs::stream::infrared));
        left_frame = cv::Mat(height, width, CV_8U);
        std::copy(left_data, left_data + height * width * left_frame.elemSize(), left_frame.data);
        cv::cvtColor(left_frame, left_frame, CV_GRAY2RGBA);

        const uint8_t* right_data = reinterpret_cast<const uint8_t*>(dev_->get_frame_data(rs::stream::infrared2));
        right_frame = cv::Mat(height, width, CV_8U);
        std::copy(right_data, right_data + height * width * right_frame.elemSize(), right_frame.data);
        cv::cvtColor(right_frame, right_frame, CV_GRAY2RGBA);

        const uint8_t* depth_data = reinterpret_cast<const uint8_t*>(dev_->get_frame_data(rs::stream::depth));
        depth_frame = cv::Mat(height, width, CV_16U);
        std::copy(depth_data, depth_data + height * width * depth_frame.elemSize(), depth_frame.data);
        depth_frame.convertTo(depth_frame, CV_32F, depth_scale_);

        return true;
    }

    ait::stereo::StereoCameraCalibration getStereoCalibration() {
        ait::stereo::StereoCameraCalibration stereo_calibration;
        stereo_calibration.image_size.width = width_;
        stereo_calibration.image_size.height = height_;
        // Intrinsics are same for infrared, infrared2 and depth (also no distortion)
        rs::intrinsics intrinsics = dev_->get_stream_intrinsics(rs::stream::infrared);
        stereo_calibration.left.camera_matrix.at<double>(0, 0) = intrinsics.fx;
        stereo_calibration.left.camera_matrix.at<double>(0, 2) = intrinsics.ppx;
        stereo_calibration.left.camera_matrix.at<double>(1, 1) = intrinsics.fy;
        stereo_calibration.left.camera_matrix.at<double>(1, 2) = intrinsics.ppy;
        stereo_calibration.left.camera_matrix.at<double>(2, 2) = 1;
        // Translation from left to right infrared camera
        rs::extrinsics extrinsics_right = dev_->get_extrinsics(rs::stream::infrared2, rs::stream::infrared);
        for (int i = 0; i < stereo_calibration.translation.rows; ++i) {
            stereo_calibration.translation.at<double>(i) = extrinsics_right.translation[i];
        }
        return stereo_calibration;
    }

private:
    unsigned int width_;
    unsigned int height_;

    rs::context ctx_;
    rs::device* dev_;
    float depth_scale_;
};

// Stereo frame retrieve function
bool retrieveFrames(ait::video::VideoSourceZED* video_ptr, double* timestamp, cv::Mat& left_frame, cv::Mat& right_frame, cv::Mat& depth_frame)
{
//  cv::Size display_size(data->video_ptr->getWidth(), data->video_ptr->getHeight());
//  cv::Mat left_grabbed_frame(display_size, CV_8UC4);
//  cv::Mat right_grabbed_frame(display_size, CV_8UC4);
//  cv::Mat depth_grabbed_frame(display_size, CV_8UC4);
    if (!video_ptr->grab()) {
        throw std::runtime_error("Failed to grab next frame.");
    }
//    *timestamp = ros::Time::now().toSec();
    using clock = std::chrono::high_resolution_clock;
    *timestamp = std::chrono::duration_cast<std::chrono::duration<double>>(clock::now().time_since_epoch()).count();
    // Retrieve stereo and depth frames
    if (!video_ptr->retrieveLeft(&left_frame)) {
        throw std::runtime_error("Failed to retrieve left frame");
    }
    if (!video_ptr->retrieveRight(&right_frame)) {
        throw std::runtime_error("Failed to retrieve right frame");
    }
    if (!video_ptr->retrieveDepthFloat(&depth_frame)) {
        throw std::runtime_error("Failed to retrieve depth frame");
    }
    // Make sure all frames have the same size and type
    if (left_frame.rows != right_frame.rows || left_frame.rows != depth_frame.rows) {
        throw std::runtime_error("Stereo and depth frames do not have the same height");
    }
    if (left_frame.cols != right_frame.cols || left_frame.cols != depth_frame.cols) {
        throw std::runtime_error("Stereo and depth frames do not have the same width");
    }
    if (left_frame.type() != right_frame.type()) {
        throw std::runtime_error("Stereo frames do not have the same type");
    }
    if (depth_frame.type() != CV_32F) {
        throw std::runtime_error("Depth frame does not have single-precision floating point type");
    }
    return true;
};

StereoCalibration convertStereoCalibration(const ait::stereo::StereoCameraCalibration& stereo_calibration)
{
    StereoCalibration stereo_sensor_calibration;
    // Image widths and heights
    stereo_sensor_calibration.color_image_width_left = stereo_calibration.image_size.width;
    stereo_sensor_calibration.color_image_height_left = stereo_calibration.image_size.height;
    stereo_sensor_calibration.color_image_width_right = stereo_calibration.image_size.width;
    stereo_sensor_calibration.color_image_height_right = stereo_calibration.image_size.height;
    stereo_sensor_calibration.depth_image_width = stereo_calibration.image_size.width;
    stereo_sensor_calibration.depth_image_height = stereo_calibration.image_size.height;
    // Left intrinsics and extrinsics
    ait::Mat4f left_intrinsics = ait::Mat4f::Identity();
    left_intrinsics.block<3, 3>(0, 0) = stereo_calibration.left.getCameraMatrixEigen().cast<float>();
    ait::Mat4f  left_extrinsics;
    left_extrinsics = stereo_calibration.getLeftExtrinsicsEigen().cast<float>();
    stereo_sensor_calibration.calibration_color_left.setMatrices(left_intrinsics, left_extrinsics);
    // Depth intrinsics and extrinsics
    stereo_sensor_calibration.calibration_depth.setMatrices(left_intrinsics, left_extrinsics);
    // Right intrinsics and extrinsics
    ait::Mat4f right_intrinsics = ait::Mat4f::Identity();
    right_intrinsics.block<3, 3>(0, 0) = stereo_calibration.right.getCameraMatrixEigen().cast<float>();
    ait::Mat4f right_extrinsics;
    right_extrinsics = stereo_calibration.getRightExtrinsicsEigen().cast<float>();
    stereo_sensor_calibration.calibration_color_right.setMatrices(right_intrinsics, right_extrinsics);

    return stereo_sensor_calibration;
}


std::pair<bool, boost::program_options::variables_map> process_commandline(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description generic_options("Allowed options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("rotate", po::bool_switch()->default_value(false), "Rotate stereo images")
        ("show", po::bool_switch()->default_value(false), "Render output")
        ("fps", po::value<double>()->default_value(15), "Frame-rate to stream")
        ;

    po::options_description zed_options("ZED options");
    zed_options.add_options()
        ("svo-file", po::value<std::string>(), "SVO file to use")
        ("mode", po::value<int>()->default_value(3), "ZED resolution mode")
        ("zed-fps", po::value<double>()->default_value(30), "Frame-rate to capture")
        ("zed-params", po::value<std::string>(), "ZED parameter file")
//        ("calib-file", po::value<std::string>()->default_value("camera_calibration_stereo.yml"), "Stereo calibration file.")
        ;

    po::options_description realsense_options("Realsense options");
    realsense_options.add_options()
        ("use-realsense", po::bool_switch()->default_value(false), "Use realsense camera")
        ;

    po::options_description network_options("Network options");
    network_options.add_options()
        ("remote-ip", po::value<std::string>()->default_value("127.0.0.1"), "Remote IP address")
        ("remote-port", po::value<int>()->default_value(1337), "Remote port")
        ("compress", po::bool_switch()->default_value(false), "Use compression")
        ;

    po::options_description frame_options("Frame options");
    frame_options.add_options()
        ("inverse-depth", po::bool_switch()->default_value(false), "Convert depth image to inverse-depth before encoding")
        ("min-depth-trunc", po::value<float>()->default_value(1.0f), "Minimum depth before it is truncated to be invalid")
        ("max-depth-trunc", po::value<float>()->default_value(12.0f), "Maximum depth before it is truncated to be invalid")
        ;

    po::options_description gstreamer_options("Gstreamer options");
    gstreamer_options.add_options()
      ("preprocess-branch", po::value<std::string>(), "Preprocessing branch description")
      ("encoder-branch", po::value<std::string>(), "Encoder branch description")
      ("display-branch", po::value<std::string>(), "Display branch description")
      ;

    po::options_description drone_options("Drone options");
    gstreamer_options.add_options()
      ("ignore-drone", po::bool_switch()->default_value(false), "Ignore ROS messages from drone")
      ;

    po::options_description options;
    options.add(generic_options);
    options.add(zed_options);
    options.add(realsense_options);
    options.add(network_options);
    options.add(frame_options);
    options.add(gstreamer_options);
    options.add(drone_options);
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
    if (vm.count("help"))
    {
        std::cout << options << std::endl;
        return std::make_pair(false, vm);
    }

    po::notify(vm);

    return std::make_pair(true, vm);
}

void initialize_ros(int argc, char** argv)
{
    ros::init(argc, argv, "video_streamer_bundlefusion_drone", ros::init_options::NoSigintHandler);
    if (!ros::ok()) {
        throw std::runtime_error("Unable to initialize ROS");
    }
}

void initialize_gstramer(int argc, char** argv)
{
    GError *gst_err;
    gboolean gst_initialized = gst_init_check(&argc, &argv, &gst_err);
    if (gst_initialized == FALSE) {
        std::cerr << "ERROR: gst_init_check failed: " << gst_err->message << std::endl;
        throw std::runtime_error("Unable to initialize gstreamer");
    }
}

int main(int argc, char** argv)
{
    namespace avo = ait::video;
    namespace po = boost::program_options;

    initialize_ros(argc, argv);

    initialize_gstramer(argc, argv);

    try
    {
        // Handle command line
        std::pair<bool, boost::program_options::variables_map> cmdline_result = process_commandline(argc, argv);
        if (!cmdline_result.first) {
            return 1;
        }
        boost::program_options::variables_map vm = std::move(cmdline_result.second);

        bool show = vm["show"].as<bool>();
        bool rotate = vm["rotate"].as<bool>();
        bool use_compression = vm["compress"].as<bool>();
        std::string remote_ip = vm["remote-ip"].as<std::string>();
        int remote_port = vm["remote-port"].as<int>();
        int zed_mode = vm["mode"].as<int>();
        bool inverse_depth = vm["inverse-depth"].as<bool>();
        float trunc_depth_min = vm["min-depth-trunc"].as<float>();
        float trunc_depth_max = vm["max-depth-trunc"].as<float>();

        // Initialize ZED camera
        VideoSourceRealsense *realsense_video_ptr = nullptr;
        avo::VideoSourceZED *video_ptr = nullptr;
        if (vm["use-realsense"].as<bool>()) {
            realsense_video_ptr = new VideoSourceRealsense();
        }
        else {
            video_ptr = new avo::VideoSourceZED();
            std::cout << "Initializing ZED camera... " << std::flush;
    // In debug compilation mode SVO files can't be opened
        #if _DEBUG
            video_ptr->open(static_cast<sl::zed::ZEDResolution_mode>(zed_mode));
        #else
            if (vm.count("svo-file")) {
                video_ptr->open(vm["svo-file"].as<std::string>());
            }
            else {
                video_ptr->open(static_cast<sl::zed::ZEDResolution_mode>(zed_mode));
            }
        #endif
            std::cout << "Done." << std::endl;
            if (vm.count("zed-fps")) {
                if (!video_ptr->setFPS(vm["zed-fps"].as<double>())) {
                    std::cerr << "Setting camera FPS failed" << std::endl;
                }
            }
            double camera_framerate = video_ptr->getFPS();
            std::cout << "Grabbing frames with " << camera_framerate << " Hz" << std::endl;
        }

        std::cout << "Registering CTRL-C signal handler" << std::endl;
        // Register CTRL-C handler
        std::signal(SIGINT, signalHandler);

        using NetworkClient = ait::BoostNetworkClientTCP;
        //using NetworkClient = ait::BoostNetworkClientUDP;

        //// Read stereo calibration
        ait::stereo::StereoCameraCalibration stereo_calibration;
        if (vm["use-realsense"].as<bool>()) {
            std::cout << "Getting camera calibration" << std::endl;
            stereo_calibration = realsense_video_ptr->getStereoCalibration();
            std::cout << "Done" << std::endl;
        }
        else {
            stereo_calibration = video_ptr->getStereoCalibration();
        }
        std::cout << "Converting camera calibration" << std::endl;
        StereoCalibration stereo_sensor_calibration = convertStereoCalibration(stereo_calibration);

        std::shared_ptr<ait::dji::RosDjiDrone> drone_ptr;
        if (!vm["ignore-drone"].as<bool>()) {
            drone_ptr = std::make_shared<ait::dji::RosDjiDrone>();
            // Wait for topics to be received
            std::cout << "Waiting for messages from drone ...";
            ros::Time future_time_500ms = ros::Time::now() + ros::Duration(0.5);
            ros::Time future_time_2000ms = ros::Time::now() + ros::Duration(2.0);
            while (true) {
                ros::spinOnce();
                bool received_all_topics= drone_ptr->getDrone().global_position.header.stamp >= future_time_500ms
                        && drone_ptr->getDrone().attitude_quaternion.header.stamp >= future_time_500ms
                        && drone_ptr->getDrone().velocity.header.stamp >= future_time_500ms;
                if (received_all_topics) {
                    break;
                }
                if (ros::Time::now() >= future_time_2000ms) {
                    std::cerr << "ERROR: Did not receive messages from drone within due time" << std::endl;
                    delete video_ptr;
                    return -1;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                std::cout << ".";
            }
            std::cout << "Done" << std::endl;
        }

        // Create manager to handle pipeline and communication
        ait::video::StereoNetworkSensorManager<NetworkClient> manager(stereo_sensor_calibration, StereoClientType::CLIENT_ZED, remote_ip, remote_port);
        if (vm.count("preprocess-branch")) {
          manager.getPipeline().setPreProcessBranchStr(vm["preprocess-branch"].as<std::string>());
        }
        if (vm.count("encoder-branch")) {
          manager.getPipeline().setEncoderBranchStr(vm["encoder-branch"].as<std::string>());
        }
        if (vm.count("display-branch")) {
          manager.getPipeline().setDisplayBranchStr(vm["display-branch"].as<std::string>());
        }
        manager.setUseCompression(use_compression);
        manager.setDepthTruncation(trunc_depth_min, trunc_depth_max);
        manager.setInverseDepth(inverse_depth);
        manager.start();

        // We try to push frames with streaming framerate into pipeline, even if camera framerate is different
        double stream_framerate = vm["fps"].as<double>();

        // We keep these outside the loop. This way the memory is not allocated on each iteration
        cv::Mat left_frame, right_frame, depth_frame;

        auto start_time = std::chrono::high_resolution_clock::now();

        std::atomic_bool terminate;
        terminate = false;

        // TODO
//        unsigned int frame_counter = 0;
//        bool pipeline_started = false;
        ait::RateCounter frame_rate_counter;
        ait::PaceMaker pace(stream_framerate);
        while (!terminate && !g_abort && ros::ok()) {
            // TODO
            // Make sure pipeline starts after some time. Otherwise we quit.
//            if (!pipeline_started) {
//                if (frame_counter >= PIPELINE_TIMEOUT_MIN_NUM_FRAMES && !manager.getPipeline().isPlaying()) {
//                    auto now = std::chrono::high_resolution_clock::now();
//                    if (now - start_time > PIPELINE_PLAYING_TIMEOUT) {
//                        std::cerr << "ERROR: Pipeline did not start playing in due time" << std::endl;
//                        break;
//                    }
//                }
//                else if (manager.getPipeline().isPlaying()) {
//                    pipeline_started = true;
//                }
//            }

//            std::cout << "Processing ROS messages" << std::endl;
            ros::spinOnce();

//            std::cout << "Getting location info" << std::endl;
            // Retrieve latest location info from drone
            StereoFrameLocationInfo location_info;
            if (drone_ptr) {
    //            dji_sdk::TimeStamp time_stamp = drone_ptr->getDrone()time_stamp;
                dji_sdk::GlobalPosition global_position = drone_ptr->getDrone().global_position;
                dji_sdk::AttitudeQuaternion attitude = drone_ptr->getDrone().attitude_quaternion;
                dji_sdk::Velocity velocity = drone_ptr->getDrone().velocity;
    //            location_info.timestamp = time_stamp.header.stamp.toSec();
                location_info.timestamp = global_position.header.stamp.toSec();
                location_info.timestamp = std::min(location_info.timestamp, attitude.header.stamp.toSec());
                location_info.timestamp = std::min(location_info.timestamp, velocity.header.stamp.toSec());
                location_info.latitude = global_position.latitude;
                location_info.longitude = global_position.longitude;
                location_info.altitude = global_position.altitude;
                location_info.velocity(0) = velocity.vx;
                location_info.velocity(1) = velocity.vy;
                location_info.velocity(2) = velocity.vz;
                location_info.angular_velocity(0) = attitude.wx;
                location_info.angular_velocity(1) = attitude.wy;
                location_info.angular_velocity(2) = attitude.wz;
                location_info.attitude_quaternion(0) = attitude.q0;
                location_info.attitude_quaternion(1) = attitude.q1;
                location_info.attitude_quaternion(2) = attitude.q2;
                location_info.attitude_quaternion(3) = attitude.q3;
    //            static double ts = location_info.timestamp;
    //            std::cout << "ts=" << location_info.timestamp - ts << ", latitude=" << location_info.latitude << ", longitude=" << location_info.longitude
    //                    << ", altitude=" << location_info.altitude << std::endl;
            }

//            std::cout << "Retrieving frames" << std::endl;
            double timestamp;
            if (vm["use-realsense"].as<bool>()) {
                realsense_video_ptr->retrieveFrames(&timestamp, left_frame, right_frame, depth_frame);
            }
            else {
                retrieveFrames(video_ptr, &timestamp, left_frame, right_frame, depth_frame);
            }

            bool discard_frame = false;
            if (std::abs(timestamp - location_info.timestamp) > LOCATION_TIMESTAMP_DIFF_THRESHOLD) {
                std::cout << "WARNING: Location timestamp deviates too much from stereo frame timestamp (dt="
                        << timestamp - location_info.timestamp << "). Discarding frame." << std::endl;
                discard_frame = true;
            }

            if (!discard_frame) {
                // For upside down stere-camera
                if (rotate) {
                    const int flipCode = -1;
#pragma omp parallel sections
                    {
                        if (rotate) {
                            cv::flip(left_frame, left_frame, flipCode);
                        }
#pragma omp section
                        if (rotate) {
                            cv::flip(right_frame, right_frame, flipCode);
                        }
#pragma omp section
                        if (rotate) {
                            cv::flip(depth_frame, depth_frame, flipCode);
                        }
                    }
                }

//                std::cout << "Pushing stereo frames" << std::endl;
                if (manager.pushNewStereoFrame(timestamp, left_frame, right_frame, depth_frame, location_info)) {
                    // TODO
//                    ++frame_counter;
                    frame_rate_counter.count();
                    double rate;
                    if (frame_rate_counter.reportRate(rate)) {
                        std::cout << "Running with " << rate << " Hz" << std::endl;
                    }
                }
                else {
                    std::cout << "Discarding frame" << std::endl;
                }

                if (show) {
                    cv::imshow("left frame", left_frame);
                    cv::imshow("right frame", right_frame);
                    cv::imshow("depth frame", depth_frame);
                    if (cv::waitKey(1) == 27) {
                        terminate = true;
                    }
                }
            }

//            std::cout << "Waiting for loop period" << std::endl;
            pace.sleep();
        }
        std::cout << "Stopping manager" << std::endl;
        manager.stop();

        SAFE_DELETE(video_ptr);
        SAFE_DELETE(realsense_video_ptr);
    }
    catch (const po::required_option& err)
    {
        std::cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << std::endl;
    }
    catch (const po::error& err)
    {
        std::cerr << "Error parsing command line: " << err.what() << std::endl;
    }
    catch (const std::exception& err)
    {
        std::cerr << "Exception: << " << err.what() << std::endl;
        throw;
    }

    return 0;
}
