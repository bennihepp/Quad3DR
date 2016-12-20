//==================================================
// stereo_capture_zed.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 9, 2016
//==================================================

#include <ctime>
#include <string>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <tclap/CmdLine.h>
#include <ait/video/video_source_zed.h>

std::string showImagesAndWaitForCommand(
    const std::vector<std::pair<cv::Mat, std::string>> &images_and_names,
    const std::vector<std::pair<std::string, char>> &commands_and_keys,
    int key_wait_delay=50)
{
  CV_Assert(!images_and_names.empty());
  CV_Assert(!commands_and_keys.empty());
  while (true)
  {
    // Show images and commands
    for (const auto &image_and_name : images_and_names)
    {
      cv::Mat image_copy = image_and_name.first.clone();

      // Put command description onto image
      cv::Size text_size = cv::getTextSize("ABC", 1, 1, 1, nullptr);
      for (int i = 0; i < commands_and_keys.size(); ++i)
      {
        const auto &command_and_key = commands_and_keys[i];
        cv::Point text_origin(20, 20 + i * (10 + text_size.height));
        std::ostringstream msg_out;
        msg_out << "Key: " << (char)command_and_key.second << ", Command: " << command_and_key.first;
        cv::putText(image_copy, msg_out.str(), text_origin, 1, 1, cv::Scalar(0,0,255));
      }

      cv::imshow(image_and_name.second, image_copy);
    }

    // Check key input
    int key = 0xff & cv::waitKey(key_wait_delay);
    if (key== 27)
    {
      return std::string();
    }
    for (const auto &command_and_key : commands_and_keys)
    {
      if (key == command_and_key.second)
      {
        return command_and_key.first;
      }
    }
  }
}

int main(int argc, char **argv)
{
  namespace avo = ait::video;

  try
  {
    TCLAP::CmdLine cmd("Stereo calibration tool", ' ', "0.1");
    TCLAP::ValueArg<int> device_arg("d", "device", "Device number to use", false, 0, "id", cmd);
    TCLAP::ValueArg<std::string> video_arg("v", "video", "Video device file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<std::string> svo_arg("", "svo", "SVO file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<int> mode_arg("", "mode", "ZED Resolution mode", false, 2, "mode", cmd);
    TCLAP::ValueArg<double> fps_arg("", "fps", "Frame-rate to capture", false, 15, "Hz", cmd);
    TCLAP::ValueArg<std::string> zed_params_arg("", "zed-params", "ZED parameter file", false, "", "filename", cmd);
    TCLAP::ValueArg<int> num_frames_arg("n", "num_frames", "Number of frames", false, 10, "integer", cmd);
    TCLAP::ValueArg<std::string> frames_prefix_arg("", "frames-prefix", "Frames prefix", false, "frame", "string", cmd);
    TCLAP::SwitchArg record_depth_arg("", "record-depth", "Record depth images", cmd, false);

    cmd.parse(argc, argv);

    clock_t prev_timestamp = std::clock();

    int num_frames = num_frames_arg.getValue();

    avo::VideoSourceZED video;

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

    if (video.setFPS(fps_arg.getValue()))
    {
      throw std::runtime_error("Unable to set ZED framerate");
    }
    std::cout << "ZED framerate: " << video.getFPS() << std::endl;

    cv::namedWindow("Left", 1);
    cv::namedWindow("Right", 1);
    cv::namedWindow("Depth", 1);

    std::ofstream list_file_left;
    std::ofstream list_file_right;
    std::ofstream list_file_depth;
    list_file_left.open(frames_prefix_arg.getValue() + "_left_list.txt");
    list_file_right.open(frames_prefix_arg.getValue() + "_right_list.txt");
    if (record_depth_arg.getValue())
    {
      list_file_depth.open(frames_prefix_arg.getValue() + "_right_list.txt");
    }

    cv::Size image_size;
    int next_i = 0;
    int i = 0;
    cv::Mat view_left_orig;
    cv::Mat view_right_orig;
    cv::Mat view_depth_orig;
    cv::Mat view_left;
    cv::Mat view_right;
    cv::Mat view_depth;
    while (i < num_frames)
    {
      video.grab();
      video.retrieveLeft(&view_left_orig);
      video.retrieveRight(&view_right_orig);

      view_left_orig.copyTo(view_left);
      view_right_orig.copyTo(view_right);

      if (record_depth_arg.getValue())
      {
        video.retrieveDepth(&view_depth_orig);
        view_depth_orig.copyTo(view_depth);
      }

      image_size = view_left.size();
      CV_Assert(image_size == view_right.size());

      std::vector<cv::Point2f> pointbuf_left;
      std::vector<cv::Point2f> pointbuf_right;

      cv::Mat view_left_gray;
      cv::Mat view_right_gray;
      cv::cvtColor(view_left, view_left_gray, cv::COLOR_BGR2GRAY);
      cv::cvtColor(view_right, view_right_gray, cv::COLOR_BGR2GRAY);

      std::string msg = "-------------------------------------------------";
      int baseLine = 0;
      cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
      cv::Point text_origin(view_left.cols - 2*textSize.width - 10, view_left.rows - 2*baseLine - 10);

      msg = cv::format("%d/%d", i, num_frames);

      cv::putText(view_left, msg, text_origin, 1, 1, cv::Scalar(0,0,255));

      cv::imshow("Left", view_left);
      cv::imshow("Right", view_right);
      if (record_depth_arg.getValue())
      {
        cv::imshow("Depth", view_depth);
      }

      int key = 0xff & cv::waitKey(10);

      if ((key & 255) == 27 || key == 'q')
      {
        break;
      }

      if (key == 'r')
      {
        std::cout << "Clearing all frames" << std::endl;
        list_file_left.close();
        list_file_right.close();
        list_file_left.open(frames_prefix_arg.getValue() + "_left_list.txt");
        list_file_right.open(frames_prefix_arg.getValue() + "_right_list.txt");
        i = 0;
      }

      if (key == 'c')
      {
        std::clock_t current_timestamp = std::clock();
        std::clock_t d_stamp = current_timestamp - prev_timestamp;
        if (d_stamp > 0.1 * CLOCKS_PER_SEC)
        {
          std::vector<std::pair<cv::Mat, std::string>> images_and_names = {{view_left, "Left"}, {view_right, "Right"}};
          if (record_depth_arg.getValue())
          {
            images_and_names.push_back({view_depth, "Depth"});
          }
          std::vector<std::pair<std::string, char>> commands_and_keys = {{"keep", 'k'}, {"discard", 'd'}};
          std::string command = showImagesAndWaitForCommand(images_and_names, commands_and_keys);
          if (command == "keep")
          {
            prev_timestamp = current_timestamp;
            std::cout << "Keeping frame " << i << std::endl;

            std::ostringstream out;
            out << frames_prefix_arg.getValue() << "_left_" << i << ".png";
            cv::imwrite(out.str(), view_left_orig);
            list_file_left << out.str() << std::endl;

            out.str("");
            out.clear();
            out << frames_prefix_arg.getValue() << "_right_" << i << ".png";
            cv::imwrite(out.str(), view_right_orig);
            list_file_right << out.str() << std::endl;

            if (record_depth_arg.getValue())
            {
              out.str("");
              out.clear();
              out << frames_prefix_arg.getValue() << "_depth_" << i << ".png";
              cv::imwrite(out.str(), view_depth_orig);
              list_file_depth << out.str() << std::endl;
            }

            ++i;
          }
          else if (command.empty())
          {
            break;
          }
        }
      }
    }
    list_file_left.close();
    list_file_right.close();
    list_file_depth.close();
  }
  catch (TCLAP::ArgException &err)
  {
    std::cerr << "Command line error: " << err.error() << " for arg " << err.argId() << std::endl;
  }

  return 0;
}
