//==================================================
// video_capture_zed.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 29, 2016
//==================================================

#include <iostream>
#include <stdexcept>
#include <tclap/CmdLine.h>
#include <opencv2/opencv.hpp>
#include <ait/video/video_source_zed.h>


int main(int argc, char **argv)
{
  namespace avo = ait::video;

  try
  {
    TCLAP::CmdLine cmd("Video capture tool", ' ', "0.1");
    TCLAP::ValueArg<int> device_arg("d", "device", "Device number to use", false, 0, "id", cmd);
    TCLAP::ValueArg<std::string> video_arg("v", "video", "Video file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<int> mode_arg("", "mode", "ZED Resolution mode", false, 2, "mode", cmd);
    TCLAP::ValueArg<double> fps_arg("", "fps", "Frame-rate to capture", false, 0, "Hz", cmd);
    TCLAP::ValueArg<bool> show_arg("s", "show", "Show captured video", false, true, "boolean", cmd);
    TCLAP::ValueArg<std::string> zed_params_arg("", "zed-params", "ZED parameter file", false, "", "filename", cmd);

    cmd.parse(argc, argv);

    avo::VideoSourceZED video;

    if (zed_params_arg.isSet())
    {
      video.getInitParameters().load(zed_params_arg.getValue());
    }
    video.open(static_cast<sl::zed::ZEDResolution_mode>(mode_arg.getValue()));
    //video.getInitParameters().save("MyParam");

    if (fps_arg.isSet())
    {
      if (video.setFPS(fps_arg.getValue()))
      {
        throw std::runtime_error("Unable to set ZED framerate");
      }
    }
    std::cout << "ZED framerate: " << video.getFPS() << std::endl;

    int width = video.getWidth();
    int height = video.getHeight();

    cv::Size display_size(width, height);
    cv::Mat disp_left(display_size, CV_8UC4);
    cv::Mat disp_right(display_size, CV_8UC4);
    cv::Mat disp_depth(display_size, CV_8UC4);

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
//      video.retrieveDisparity(&disp_depth);
//      video.retrieveConfidence(&disp_depth);

      if (show_arg.getValue())
      {
        cv::imshow("left", disp_left);
        cv::imshow("right", disp_right);
        cv::imshow("depth", disp_depth);
//        cv::imshow("left", sl::zed::slMat2cvMat(left));
//        cv::imshow("right", sl::zed::slMat2cvMat(right));
//        cv::imshow("depth", sl::zed::slMat2cvMat(depth));
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

      if (show_arg.getValue())
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
