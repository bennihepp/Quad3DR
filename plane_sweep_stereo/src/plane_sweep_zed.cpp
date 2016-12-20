//==================================================
// plane_sweep_zed.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 23, 2016
//==================================================

#include <iostream>
#include <map>
#include <boost/program_options.hpp>
#include <Eigen/Dense>
#include <opencv2/highgui/highgui.hpp>
#include <psl_base/exception.h>
#include <psl_base/cameraMatrix.h>
#include <psl_stereo/cudaPlaneSweep.h>
#include <ait/stereo/stereo_calibration.h>
#include <ait/utilities.h>
#include <ait/video/video_source_zed.h>

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;
  namespace ast = ait::stereo;
  namespace avo = ait::video;

  try
  {
    bool hide = false;
    int draw_period = 1;

    po::options_description generic_options("Allowed options");
    generic_options.add_options()
            ("help", "Produce help message")
            ("hide", po::bool_switch(&hide)->default_value(false), "Do not render output")
            ("draw-period", po::value<int>(&draw_period)->default_value(5), "Period of drawing frames")
            ;

    po::options_description zed_options("ZED options");
    zed_options.add_options()
        ("device-id", po::value<int>()->default_value(0), "Device number to use")
        ("video-device", po::value<std::string>(), "Video device file to use")
        ("svo-file", po::value<std::string>(), "SVO file to use")
        ("mode", po::value<int>()->default_value(2), "ZED resolution mode")
        ("fps", po::value<double>()->default_value(0), "Frame-rate to capture")
        ("zed-params", po::value<std::string>(), "ZED parameter file")
        ("calib-file", po::value<std::string>()->default_value("camera_calibration_stereo.yml"), "Stereo calibration file.")
        ;

    po::options_description options;
    options.add(generic_options);
    options.add(zed_options);
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
    if (vm.count("help"))
    {
        std::cout << options << std::endl;
        return 1;
    }

    po::notify(vm);

    // Initialize ZED camera object
    avo::VideoSourceZED video(sl::zed::STANDARD, false, false, false);

    if (vm.count("zed-params"))
    {
      video.getInitParameters().load(vm["zed-params"].as<std::string>());
    }
//    video.getInitParameters().disableSelfCalib = false;
    if (vm.count("svo-file"))
    {
      video.open(vm["svo-file"].as<std::string>());
    }
    else
    {
      video.open(static_cast<sl::zed::ZEDResolution_mode>(vm["mode"].as<int>()));
    }

    if (vm.count("fps"))
    {
      if (video.setFPS(vm["fps"].as<double>()))
      {
        throw std::runtime_error("Unable to set ZED framerate");
      }
    }
    std::cout << "ZED framerate: " << video.getFPS() << std::endl;

    // Try to read stereo camera calibration
    ast::StereoCameraCalibration calib = ast::StereoCameraCalibration::readStereoCalibration(vm["calib-file"].as<std::string>());
    Eigen::Matrix3d K_left = calib.left.getCameraMatrixEigen();
    Eigen::Matrix3d R_left = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T_left = Eigen::Vector3d::Zero();
    Eigen::Matrix3d K_right = calib.right.getCameraMatrixEigen();
    Eigen::Matrix3d R_right = calib.getRotationEigen();
    Eigen::Vector3d T_right = calib.getTranslationEigen();

    // Create PlaneSweepLib cameras
    PSL::CameraMatrix<double> camera_left;
    PSL::CameraMatrix<double> camera_right;
    camera_left.setKRT(K_left, R_left, T_left);
    camera_right.setKRT(K_right, R_right, T_right);

    std::cout << "Left center:" << std::endl << camera_left.getC() << std::endl;
    std::cout << "Right center:" << std::endl << camera_right.getC() << std::endl;

    Eigen::Vector3d distance = camera_left.getC() - camera_right.getC();
    double avg_distance = distance.norm();
    //    float minZ = 0.5f * 1000;
    //    float maxZ = 5.0f * 1000;
    float minZ = (float) (2.5f*avg_distance);
    float maxZ = (float) (100.0f*avg_distance);
    std::cout << "minZ=" << minZ << ", maxZ=" << maxZ << std::endl;
    minZ = 0.2f;
    maxZ = 20.0f;
    std::cout << "minZ=" << minZ << ", maxZ=" << maxZ << std::endl;

    double scale = 1.0;
    int window_size = 25;
    int num_planes = 128;

    int width = video.getWidth();
    int height = video.getHeight();

    // Create display windows
    cv::Size display_size(width, height);
    cv_cuda::GpuMat img_left_gpu;
    cv_cuda::GpuMat img_right_gpu;
//    cv_cuda::GpuMat zed_depth_img_gpu;
    cv_cuda::GpuMat planesweep_depth_img_gpu;

    bool opengl_supported = false;
    if (!hide)
    {
      try
      {
        cv::namedWindow("left", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
        cv::namedWindow("right", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
        cv::namedWindow("zed_depth", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
        cv::namedWindow("planesweep_depth", cv::WINDOW_AUTOSIZE | cv::WINDOW_OPENGL);
        opengl_supported = true;
      }
      catch (const cv::Exception &err)
      {
        cv::namedWindow("left", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("right", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("zed_depth", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("planesweep_depth", cv::WINDOW_AUTOSIZE);
      }
    }

    cv_cuda::Stream stream;
    ait::RateCounter rate_counter;
    int key = -1;
    while (key != 27)
    {
      if (!video.grab())
      {
        throw std::runtime_error("Failed to grab frame from camera");
      }
      video.retrieveLeftGpu(&img_left_gpu, false);
      video.retrieveRightGpu(&img_right_gpu, false);
//      video.retrieveDepthGpu(&zed_depth_img_gpu, false);
//      video.retrieveDepthFloatGpu(&zed_depth_img_gpu, false);
//      video.retrieveDisparityFloatGpu(&disparity_img_gpu, false);
//      video.retrieveConfidenceFloatGpu(&confidence_img_gpu, false);

      // Set up plane-sweep computation
      PSL::CudaPlaneSweep cPS;
      cPS.setScale(scale); // Scale the images down to 0.25 times the original side length
      cPS.setZRange(minZ, maxZ);
      cPS.setMatchWindowSize(window_size, window_size);
      cPS.setNumPlanes(num_planes);
      cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_NONE);
      cPS.setPlaneGenerationMode(PSL::PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
      cPS.setSubPixelInterpolationMode(PSL::PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
      cPS.enableOutputBestDepth();
      cPS.enableOutputBestCosts(false);
      cPS.enableOuputUniquenessRatio(false);
      cPS.enableOutputCostVolume(false);
      cPS.enableSubPixel();

      // Download images to CPU
      cv::Mat img_left;
      cv::Mat img_right;
      img_left_gpu.download(img_left, stream);
      img_right_gpu.download(img_right, stream);
      stream.waitForCompletion();

      {
        // Perform matching with Color SAD
        cPS.enableColorMatching();
        cPS.setMatchingCosts(PSL::PLANE_SWEEP_SAD);

        // Uload the images
        ait::ProfilingTimer timer;
        int id_left = cPS.addImage(img_left, camera_left);
        int id_right = cPS.addImage(img_right, camera_right);
        int ref_id = id_left;
        timer.stopAndPrintTiming("Uploading images for Plane Sweep computation");

        // Perform Plane-Sweep
        timer = ait::ProfilingTimer();
        cPS.process(ref_id);
        PSL::DepthMap<float, double> dM;
        dM = cPS.getBestDepth();
        cv::Mat refImage = cPS.downloadImage(ref_id);
        timer.stopAndPrintTiming("Plane sweep stereo with color SAD");
        if (!hide && rate_counter.getCounts() % draw_period == 0)
        {
//          dM.display(minZ, maxZ, 1, "color_sad_depth");
//          dM.displayInvDepth(minZ, maxZ, 1, "color_sad_depth");
          //          dM.displayInvDepthColored(minZ, maxZ, 1, "Color SAD");
          cv::Mat depth_img(dM.getHeight(), dM.getWidth(), CV_32F, dM.getDataPtr());
          depth_img.setTo(0, depth_img > 0.75 * maxZ);
          double min, max;
          cv::minMaxIdx(depth_img, &min, &max);
          std::cout << "Color SAD depth: min=" << min << ", max=" << max << std::endl;
          double alpha = 255.0 / (max - min);
          double beta = - alpha * min;
          depth_img.convertTo(depth_img, CV_8U, alpha, beta);
          cv::imshow("color_sad_depth", depth_img);
          cv::Mat depth_hist = ait::Utilities::drawHistogram(depth_img, 100);
          cv::imshow("depth hist", depth_hist);
        }
      }

//      img_left = ait::Utilities::convertToGrayscale(img_left);
//      img_right= ait::Utilities::convertToGrayscale(img_right);
//
//      {
//        // Perform matching with Grayscale SAD
//        cPS.enableColorMatching(false);
//        cPS.setMatchingCosts(PSL::PLANE_SWEEP_SAD);
//
//        // Uload the images
//        ait::ProfilingTimer timer;
//        int id_left = cPS.addImage(img_left, camera_left);
//        int id_right = cPS.addImage(img_right, camera_right);
//        int ref_id = id_left;
//        timer.stopAndPrintTiming("Uploading images for Plane Sweep computation");
//
//        // Perform Plane-Sweep
//        timer = ait::ProfilingTimer();
//        cPS.process(ref_id);
//        PSL::DepthMap<float, double> dM;
//        dM = cPS.getBestDepth();
//        cv::Mat refImage = cPS.downloadImage(ref_id);
//        timer.stopAndPrintTiming("Plane sweep stereo with grayscale SAD");
//        if (!hide && rate_counter.getCounts() % draw_period == 0)
//        {
//          dM.displayInvDepthColored(minZ, maxZ, 100, "Grayscale SAD");
//        }
//      }
//
//      {
//        // Perform matching with Grayscale ZNCC
//        cPS.enableColorMatching(false);
//        cPS.setMatchingCosts(PSL::PLANE_SWEEP_ZNCC);
//
//        // Uload the images
//        ait::ProfilingTimer timer;
//        int id_left = cPS.addImage(img_left, camera_left);
//        int id_right = cPS.addImage(img_right, camera_right);
//        int ref_id = id_left;
//        timer.stopAndPrintTiming("Uploading images for Plane Sweep computation");
//
//        // Perform Plane-Sweep
//        timer = ait::ProfilingTimer();
//        cPS.process(ref_id);
//        PSL::DepthMap<float, double> dM;
//        dM = cPS.getBestDepth();
//        cv::Mat refImage = cPS.downloadImage(ref_id);
//        timer.stopAndPrintTiming("Plane sweep stereo with grayscale ZNCC");
//        if (!hide && rate_counter.getCounts() % draw_period == 0)
//        {
//          dM.displayInvDepthColored(minZ, maxZ, 100, "Grayscale ZNCC");
//        }
//      }

      // Optional draw results
      if (!hide && rate_counter.getCounts() % draw_period == 0)
      {
        if (opengl_supported)
        {
          cv::imshow("left", img_left_gpu);
          cv::imshow("right", img_right_gpu);
//          cv::imshow("zed_depth", zed_depth_img_gpu);
//          cv::Mat zed_depth_img;
//          zed_depth_img_gpu.download(zed_depth_img, stream);
//          cv::extractChannel(zed_depth_img, zed_depth_img, 0);
//          cv::imshow("zed_depth", ait::Utilities::drawImageWithColormap(zed_depth_img));
        }
        else
        {
          cv::Mat img_left;
          cv::Mat img_right;
          cv::Mat zed_depth_img;
          img_left_gpu.download(img_left, stream);
          img_right_gpu.download(img_right, stream);
//          zed_depth_img_gpu.download(zed_depth_img, stream);
//          stream.waitForCompletion();
          cv::imshow("left", img_left);
          cv::imshow("right", img_right);
//          cv::imshow("zed_depth", zed_depth_img);
        }
      }

      rate_counter.increment();
      if (rate_counter.getElapsedTime() >= 1.0)
      {
        std::cout << "Running with " << rate_counter.getRateAndReset() << " Hz" << std::endl;
      }

      if (!hide)
      {
        key = cv::waitKey(10) & 0xff;
      }

    }  // while (key != 27)

  }
  catch (const po::required_option &err)
  {
    std::cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << std::endl;
  }
  catch (const po::error &err)
  {
    std::cerr << "Error parsing command line: " << err.what() << std::endl;
  }

  return 0;
}
