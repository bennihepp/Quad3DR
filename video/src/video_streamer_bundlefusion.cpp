//==================================================
// video_streamer_bundlefusion.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 7, 2016
//==================================================

#define SIMULATE_ZED 0
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

#include <ait/common.h>
#include <ait/math.h>
#include <ait/video/video_source_zed.h>
#include <ait/video/StereoNetworkSensorManager.h>
#include <ait/video/EncodingGstreamerPipeline.h>

// Test direct depth compression (FastLZ, zlib, Snappy)
//#include <fastlz/fastlz.h>
//#include <snappy/snappy.h>
//#include <zlib.h>

volatile bool g_abort;

void signalHandler(int sig)
{
    g_abort = true;
    std::cout << "Received CTRL-C. Aborting" << std::endl;
}

// Stereo frame retrieve function
bool retrieveFrame(ait::video::VideoSourceZED* video_ptr, double* timestamp, cv::Mat& left_frame, cv::Mat& right_frame, cv::Mat& depth_frame)
{
    using clock = std::chrono::high_resolution_clock;
//  cv::Size display_size(data->video_ptr->getWidth(), data->video_ptr->getHeight());
//  cv::Mat left_grabbed_frame(display_size, CV_8UC4);
//  cv::Mat right_grabbed_frame(display_size, CV_8UC4);
//  cv::Mat depth_grabbed_frame(display_size, CV_8UC4);
    if (!video_ptr->grab()) {
        throw std::runtime_error("Failed to grab next frame.");
    }
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

    po::options_description options;
    options.add(generic_options);
    options.add(zed_options);
    options.add(network_options);
    options.add(frame_options);
    options.add(gstreamer_options);
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

void initialize_gstramer(int argc, char** argv)
{
    GError *gst_err;
    gboolean gst_initialized = gst_init_check(&argc, &argv, &gst_err);
    if (gst_initialized == FALSE) {
        std::cerr << "ERROR: gst_init_check failed: " << gst_err->message << std::endl;
        throw std::runtime_error("Unable to initialize gstreamer");
    }
}

const cv::Mat& convertDepthFrameFloatToUint16(const cv::Mat& depth_frame)
{
    // Convert float to uint16_t depth image (scaled by 1000)
    static cv::Mat depth_frame_uint16;
    if (depth_frame_uint16.empty()
        || depth_frame_uint16.rows != depth_frame.rows
        || depth_frame_uint16.cols != depth_frame.cols) {
        depth_frame_uint16 = cv::Mat(depth_frame.rows, depth_frame.cols, CV_16S);
    }
#pragma omp parallel for
    for (int i = 0; i < depth_frame.rows * depth_frame.cols; ++i) {
        float depth = depth_frame.at<float>(i);
        if (std::isfinite(depth)) {
            depth_frame_uint16.at<uint16_t>(i) = static_cast<uint16_t>(depth * 1000);
        }
        else {
            depth_frame_uint16.at<uint16_t>(i) = 0.0;
        }
    }
    return depth_frame_uint16;
}

int main(int argc, char** argv)
{
    namespace avo = ait::video;
    namespace po = boost::program_options;

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
        double camera_framerate;
        avo::VideoSourceZED *video_ptr = new avo::VideoSourceZED();
        std::cout << "Initializing ZED camera... " << std::flush;
#if SIMULATE_ZED
        camera_framerate = 15;
#else
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
        camera_framerate = video_ptr->getFPS();
        std::cout << "Grabbing frames with " << camera_framerate << " Hz" << std::endl;
#endif

        // Register CTRL-C handler
        std::signal(SIGINT, signalHandler);

        using NetworkClient = ait::BoostNetworkClientTCP;
        //using NetworkClient = ait::BoostNetworkClientUDP;

#if SIMULATE_ZED
        StereoCalibration stereo_sensor_calibration;
        memset(&stereo_sensor_calibration, 0, sizeof(stereo_sensor_calibration));
        stereo_sensor_calibration.color_image_width_left = 640;
        stereo_sensor_calibration.color_image_width_right = 640;
        stereo_sensor_calibration.depth_image_width = 640;
        stereo_sensor_calibration.color_image_height_left = 360;
        stereo_sensor_calibration.color_image_height_right = 360;
        stereo_sensor_calibration.depth_image_height = 360;
#else
        //// Read stereo calibration
        ait::stereo::StereoCameraCalibration stereo_calibration = video_ptr->getStereoCalibration();
        StereoCalibration stereo_sensor_calibration = convertStereoCalibration(stereo_calibration);
#endif

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

        // TODO
//        std::chrono::seconds wait_for_playing_timeout(3);
//        auto start_time = std::chrono::high_resolution_clock::now();

        std::atomic_bool terminate;
        terminate = false;

        ait::RateCounter frame_rate_counter;
		ait::PaceMaker pace(stream_framerate);
        while (!terminate && !g_abort) {
            // TODO
            // Make sure pipeline starts after some time. Otherwise we quit.
//            if (!manager.getPipeline().isPlaying()) {
//                auto now = std::chrono::high_resolution_clock::now();
//                if (now - start_time > wait_for_playing_timeout) {
//                    std::cerr << "ERROR: Pipeline did not start playing in due time" << std::endl;
//                    break;
//                }
//            }

#if !SIMULATE_ZED
            double timestamp;
            retrieveFrame(video_ptr, &timestamp, left_frame, right_frame, depth_frame);

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

//            // Test direct depth compression (FastLZ, zlib, Snappy)
//            using clock = std::chrono::high_resolution_clock;
//            cv::Mat depth_frame_uint16 = convertDepthFrameFloatToUint16(depth_frame);
//            if (!prev_depth_frame_uint16.empty()) {
//                size_t input_size = depth_frame_uint16.rows * depth_frame_uint16.cols * depth_frame_uint16.elemSize();
//                size_t output_size = static_cast<size_t>(input_size * 1.1);
////                size_t output_size = snappy::MaxCompressedLength(input_size);
//                uint8_t* output = new uint8_t[output_size];
//                auto start_time = clock::now();
//    //            size_t compressed_size = static_cast<int>(fastlz_compress_level(1, depth_frame_uint16.data, input_size, output));
//                size_t compressed_size;
////                snappy::RawCompress((const char*)dframe.data, input_size, (char*)output, &compressed_size);
//                int level = 5;
//                compress2((Bytef*)output, &compressed_size, (const Bytef*)depth_frame_uint16.data, input_size, level);
//                auto stop_time = clock::now();
//                delete[] output;
//                auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time - start_time).count();
//                std::cout << "Compressed " << input_size/1024. << " kB to " << compressed_size/1024. << " kB in " << dt << " ms" << std::endl;
//            }

            if (manager.pushNewStereoFrame(timestamp, left_frame, right_frame, depth_frame)) {
                frame_rate_counter.count();
                double rate;
                if (frame_rate_counter.reportRate(rate)) {
                    std::cout << "Running with " << rate << " Hz" << std::endl;
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
            else {
                std::cout << "Discarding frame" << std::endl;
            }
#endif

            pace.sleep();
        }
        manager.stop();

        delete video_ptr;
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
