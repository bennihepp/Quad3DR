//==================================================
// sparse_stereo_zed.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 2, 2016
//==================================================

#include <iostream>
#include <stdexcept>
#include <vector>
#include <tclap/CmdLine.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/sfm.hpp>

#include <opencv2/cudafeatures2d.hpp>
#include <opencv2/xfeatures2d/cuda.hpp>

#include <video_source_zed.h>
#include <sparse_stereo_matcher.h>
#include <utilities.h>


template <typename T>
std::vector<cv::Point3d> sparseStereoMatching(
    const stereo::SparseStereoMatcher<T> &matcher,
    cv::InputArray left_input_img, cv::InputArray right_input_img,
    std::vector<cv::Point2d> *image_points=nullptr)
{
  stereo::Timer timer = stereo::Timer();
  std::vector<cv::Point3d> points_3d = matcher.match(left_input_img, right_input_img, image_points);
  timer.stopAndPrintTiming("sparse stereo matching");
  return points_3d;
}

int main(int argc, char **argv)
{
  try
  {
    TCLAP::CmdLine cmd("Sparse stereo matching", ' ', "0.1");
    TCLAP::ValueArg<int> device_arg("d", "device", "Device number to use", false, 0, "id", cmd);
    TCLAP::ValueArg<std::string> video_arg("v", "video", "Video device file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<std::string> svo_arg("", "svo", "SVO file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<int> mode_arg("", "mode", "ZED Resolution mode", false, 2, "mode", cmd);
    TCLAP::ValueArg<double> fps_arg("", "fps", "Frame-rate to capture", false, 0, "Hz", cmd);
    TCLAP::SwitchArg hide_arg("", "hide", "Hide captured video", cmd, false);
    TCLAP::ValueArg<std::string> calib_arg("c", "calib", "Calibration file to use", false, "camera_calibration_stereo.yml", "filename", cmd);
    TCLAP::ValueArg<std::string> zed_params_arg("", "zed-params", "ZED parameter file", false, "", "filename", cmd);

    cmd.parse(argc, argv);

    // TODO
    cv::cuda::printCudaDeviceInfo(cv::cuda::getDevice());
//    cv::cuda::printShortCudaDeviceInfo(cv::cuda::getDevice());

    video::VideoSourceZED video;

    if (zed_params_arg.isSet())
    {
      video.getInitParameters().load(zed_params_arg.getValue());
    }
//    video.getInitParameters().disableSelfCalib = false;
    if (svo_arg.isSet())
    {
      video.open(svo_arg.getValue());
    }
    else
    {
      video.open(static_cast<sl::zed::ZEDResolution_mode>(mode_arg.getValue()));
    }
    video.getInitParameters().save("MyParam");

    if (fps_arg.isSet())
    {
      if (video.setFPS(fps_arg.getValue()))
      {
        throw std::runtime_error("Unable to set ZED framerate");
      }
    }
    std::cout << "ZED framerate: " << video.getFPS() << std::endl;

    stereo::StereoCameraCalibration calib = stereo::Utilities::readStereoCalibration(calib_arg.getValue());

    // SURF CUDA
//    using FeatureType = cv::cuda::SURF_CUDA;
//    cv::Ptr<FeatureType> feature_computer = new FeatureType();

    // SURF
//    using DetectorType = cv::xfeatures2d::SURF;
//    using DescriptorType = cv::xfeatures2d::SURF;
//    const int hessian_threshold = 400;
//    cv::Ptr<DescriptorType> detector = FeatureType::create(hessian_threshold);

    // ORB
//    using DetectorType = cv::ORB;
//    using DescriptorType = cv::ORB;
//    cv::Ptr<DetectorType> detector = DetectorType::create();
//    cv::Ptr<DescriptorType> descriptor_computer = detector;

    // ORB CUDA
//    using FeatureType = cv::cuda::ORB;
//    cv::Ptr<FeatureType> feature_computer = FeatureType::create(500, 1.2f, 8);

    // FREAK
    using DetectorType = cv::FastFeatureDetector;
    using DescriptorType = cv::xfeatures2d::FREAK;
    cv::Ptr<DetectorType> detector = DetectorType::create(20, true);
    cv::Ptr<DetectorType> detector_2 = DetectorType::create(20, true);
    cv::Ptr<DescriptorType> descriptor_computer = DescriptorType::create();
    cv::Ptr<DescriptorType> descriptor_computer_2 = DescriptorType::create();

    // Create feature detector
//    cv::Ptr<DetectorType> detector_2 = detector;
//    cv::Ptr<DescriptorType> descriptor_computer_2 = descriptor_computer;
    using FeatureDetectorType = stereo::FeatureDetectorOpenCV<DetectorType, DescriptorType>;
    cv::Ptr<FeatureDetectorType> feature_detector = new FeatureDetectorType(detector, detector_2, descriptor_computer, descriptor_computer_2);
//    using FeatureDetectorType = stereo::FeatureDetectorOpenCVSurfCuda<FeatureType>;
//    cv::Ptr<FeatureDetectorType> feature_detector = new FeatureDetectorType(feature_computer);
//      using FeatureDetectorType = stereo::FeatureDetectorOpenCVCuda<FeatureType>;
//      cv::Ptr<FeatureDetectorType> feature_detector = new FeatureDetectorType(feature_computer);

    // Create sparse matcher
    using SparseStereoMatcherType = stereo::SparseStereoMatcher<FeatureDetectorType>;
    SparseStereoMatcherType matcher(feature_detector, calib);

    // ORB
//    matcher.setFlannIndexParams(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
//    matcher.setMatchNorm(cv::NORM_HAMMING2);

    // FREAK
    //    matcher.setFlannIndexParams(cv::makePtr<cv::flann::LshIndexParams>(20, 10, 2));
    matcher.setMatchNorm(cv::NORM_HAMMING);

    // General
    feature_detector->setMaxNumOfKeypoints(500);
    matcher.setRatioTestThreshold(1.0);
    matcher.setEpipolarConstraintThreshold(1.0);

    int width = video.getWidth();
    int height = video.getHeight();

    cv::Size display_size(width, height);
    cv::Mat disp_left(display_size, CV_8UC4);
    cv::Mat disp_right(display_size, CV_8UC4);
    cv::Mat disp_depth(display_size, CV_8UC4);
    cv::Mat depth_image(display_size, CV_32FC1);

    cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("right", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);

    int64_t start_ticks = cv::getTickCount();
    int frame_counter = 0;
    int key = -1;
    while (key != 27)
    {
      if (!video.grab())
      {
        throw std::runtime_error("Failed to grab frame from camera");
      }
      video.retrieveLeft(&disp_left);
      video.retrieveRight(&disp_right);
      video.retrieveDepth(&disp_depth);
      video.retrieveDepthFloat(&depth_image);
//      video.retrieveDisparity(&disp_depth);
//      video.retrieveConfidence(&disp_depth);

      if (!hide_arg.getValue())
      {
        cv::imshow("left", disp_left);
        cv::imshow("right", disp_right);
        cv::imshow("depth", disp_depth);
//        cv::imshow("left", sl::zed::slMat2cvMat(left));
//        cv::imshow("right", sl::zed::slMat2cvMat(right));
//        cv::imshow("depth", sl::zed::slMat2cvMat(depth));
      }

      try
      {
        std::vector<cv::Point2d> image_points;
        std::vector<cv::Point3d> points_3d = sparseStereoMatching(matcher, disp_left, disp_right, &image_points);

//        for (int i = 0; i < points_3d.size(); ++i)
//        {
//          double depth = points_3d[i].z / 1000.0;
//          int x = static_cast<int>(image_points[i].x);
//          int y = static_cast<int>(image_points[i].y);
//          float depth_zed = depth_image.at<float>(y, x);
//          if (!std::isnan(depth_zed))
//          {
//            double diff = depth - depth_zed;
//            std::cout << "diff=" << diff << ", depth=" << depth << std::endl;
//          }
//        }
      }
      catch (const SparseStereoMatcherType::Error &err)
      {
        std::cerr << "Exception during stereo matching: " << err.what() << std::endl;
      }

      ++frame_counter;
      int64_t ticks = cv::getTickCount();
      double dt = double(ticks - start_ticks) / cv::getTickFrequency();
      double fps = frame_counter / dt;
      if (frame_counter > 30)
      {
//        std::cout << "Frame size: " << frame.cols << "x" << frame.rows << std::endl;
        std::cout << "Running with " << fps << std::endl;
        start_ticks = ticks;
        frame_counter = 0;
      }

      if (!hide_arg.getValue())
      {
        key = cv::waitKey(10);
      }
    }
  }
  catch (TCLAP::ArgException &err)
  {
    std::cerr << "Command line error: " << err.error() << " for arg " << err.argId() << std::endl;
  }

  return 0;
}
