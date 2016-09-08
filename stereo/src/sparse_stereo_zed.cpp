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
#include <video_source_zed.h>
#include <sparse_stereo_matcher.h>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/sfm.hpp>


cv::Mat undistortPoints(cv::InputArray &points, cv::InputArray camera_matrix, cv::InputArray dist_coefficients)
{
  cv::InputArray R = cv::noArray();
  cv::InputArray P = camera_matrix;
  cv::Mat undistorted_points;
  cv::undistortPoints(points, undistorted_points, camera_matrix, dist_coefficients, R, P);
  return undistorted_points;
}

cv::Mat getCameraMatrixFromZED(sl::zed::CamParameters &params)
{
  return cv::Mat();
}

cv::Mat computeEssentialMatrixFromZED(sl::zed::Camera &zed)
{
  return cv::Mat();
}

cv::Mat computeFundamentalMatrixFromZED(sl::zed::Camera &zed)
{
  return cv::Mat();
}

void print_camera_parameters(const sl::zed::CamParameters &params)
{
  std::cout << "fx=" << params.fx << ", fy=" << params.fy << ", cx=" << params.cx << ", cy=" << params.cy << std::endl;
  std::cout << "distortion=[";
  for (int i=0; i < 5; ++i)
  {
    if (i > 0)
    {
      std::cout << ", ";
    }
    std::cout << params.disto[i];
  }
  std::cout << "]" << std::endl;
  std::cout << "vFOV=" << params.vFOV << ", hFOV=" << params.hFOV << ", dFOV=" << params.dFOV << std::endl;
}

void print_stereo_parameters(const sl::zed::StereoParameters *stereo_params)
{
  std::cout << "baseline=" << stereo_params->baseline << std::endl;
  std::cout << "convergence=" << stereo_params->convergence << std::endl;
  std::cout << "Ty=" << stereo_params->Ty << ", Tz=" << stereo_params->Tz << std::endl;
  std::cout << "Rx=" << stereo_params->Rx << ", Rz=" << stereo_params->Rz << std::endl;
  std::cout << "Left camera parameters:" << std::endl;
  print_camera_parameters(stereo_params->LeftCam);
  std::cout << "Right camera parameters:" << std::endl;
  print_camera_parameters(stereo_params->RightCam);
}

template <typename T>
cv::Mat compute_feature_descriptors(const cv::Ptr<T> &detector, cv::InputArray img, std::vector<cv::KeyPoint> &keypoints)
{
  cv::Mat descriptors;
  detector->compute(img, keypoints, descriptors);
  return descriptors;
}

template <typename T>
std::vector<cv::KeyPoint> detect_keypoints(const cv::Ptr<T> &detector, cv::InputArray img)
{
  std::vector<cv::KeyPoint> keypoints;
  detector->detect(img, keypoints);
  return keypoints;
}

std::vector<cv::DMatch> match_features_bf(cv::InputArray left_descriptors, cv::InputArray right_descriptors)
{
//   Brute-Force matching
  cv::BFMatcher matcher(cv::NORM_L2);
  std::vector<cv::DMatch> matches;
  matcher.match(left_descriptors, right_descriptors, matches);
  return matches;
}

std::vector<cv::DMatch> match_features_flann(cv::InputArray left_descriptors, cv::InputArray right_descriptors)
{
  // FLANN matching
  auto index_params = cv::makePtr<cv::flann::KDTreeIndexParams>(5);
  auto search_param = cv::makePtr<cv::flann::SearchParams>(50, 0, true);
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> all_matches;
  matcher.match(left_descriptors, right_descriptors, all_matches);

  double max_dist = -std::numeric_limits<double>::infinity();
  double min_dist = std::numeric_limits<double>::infinity();
  // Quick calculation of max and min distances between keypoints
  for (int i = 0; i < left_descriptors.rows(); i++)
  {
    double dist = all_matches[i].distance;
    if (dist < min_dist)
    {
      min_dist = dist;
    }
    if (dist > max_dist)
    {
      max_dist = dist;
    }
  }
  std::cout << "-- Max dist: " << max_dist << ", Min dist: " << min_dist << std::endl;
  // Keep only "good" matches (i.e. whose distance is less than 2*min_dist,
  // or a small arbitary value in case that min_dist is very small).
  std::vector<cv::DMatch> good_matches;
  double good_threshold = std::max(5 * min_dist, 0.02);
  for (int i = 0; i < left_descriptors.rows(); i++)
  {
    if (all_matches[i].distance <= good_threshold)
    {
      good_matches.push_back(all_matches[i]);
    }
  }
  return good_matches;
}

std::vector<cv::DMatch> match_features_flann_knn2(cv::InputArray left_descriptors, cv::InputArray right_descriptors)
{
  // FLANN matching with kNN (k=2)
  int k = 2;
  auto index_params = cv::makePtr<cv::flann::KDTreeIndexParams>(5);
  auto search_param = cv::makePtr<cv::flann::SearchParams>(50, 0, true);
  cv::FlannBasedMatcher matcher;
  std::vector<std::vector<cv::DMatch>> all_matches;
  matcher.knnMatch(left_descriptors, right_descriptors, all_matches, k);

  // Lowe's ratio test
  double ratio_test_threshold = 0.7;
  std::vector<cv::DMatch> good_matches;
  for (auto knn_matches : all_matches)
  {
    if (knn_matches[0].distance < ratio_test_threshold * knn_matches[1].distance)
    {
      good_matches.push_back(knn_matches[0]);
    }
  }
  return good_matches;
}

void get_stereo_RT(
    const sl::zed::StereoParameters *params,
    cv::OutputArray left_R, cv::OutputArray left_t,
    cv::OutputArray right_R, cv::OutputArray right_t)
{
  left_R.create(3, 3, CV_64FC1);
  left_t.create(3, 1, CV_64FC1);
  right_R.create(3, 3, CV_64FC1);
  right_t.create(3, 1, CV_64FC1);
  cv::Mat left_R_mat = left_R.getMat();
  cv::Mat left_t_mat = left_t.getMat();
  cv::Mat right_R_mat = right_R.getMat();
  cv::Mat right_t_mat = right_t.getMat();

  left_R_mat = cv::Mat::eye(3, 3, CV_64FC1);
  left_t_mat = cv::Mat::zeros(3, 1, CV_64FC1);

  cv::Mat rot_x_aa = cv::Mat::zeros(3, 1, CV_64FC1);
  rot_x_aa.at<double>(0, 0) = params->Rx;
  cv::Mat rot_x;
  cv::Rodrigues(rot_x_aa, rot_x);
//  std::cout << "rot_x: " << std::endl << rot_x << std::endl;
  cv::Mat rot_z_aa = cv::Mat::zeros(3, 1, CV_64FC1);
  rot_z_aa.at<double>(2, 0) = params->Rz;
  cv::Mat rot_z;
  cv::Rodrigues(rot_z_aa, rot_z);
//  std::cout << "rot_z: " << std::endl << rot_z << std::endl;
  // TODO: Convergence
  right_R_mat = rot_x * rot_z;
  right_t_mat.at<double>(0, 1) = params->Ty;
  right_t_mat.at<double>(0, 2) = params->Tz;

  right_t_mat.at<double>(0, 0) = params->baseline;
  right_t_mat.at<double>(0, 1) = params->Ty;
  right_t_mat.at<double>(0, 2) = params->Tz;
}

cv::Mat get_camera_matrix(const sl::zed::CamParameters &params)
{
  cv::Mat camera_matrix = cv::Mat::zeros(3, 3, CV_64FC1);
  camera_matrix.at<double>(0, 0) = params.fx;
  camera_matrix.at<double>(1, 1) = params.fy;
  camera_matrix.at<double>(0, 2) = params.cx;
  camera_matrix.at<double>(1, 2) = params.cy;
  camera_matrix.at<double>(2, 2) = 1;
  return camera_matrix;
}

cv::Mat get_distortion_coefficients(const sl::zed::CamParameters &params)
{
  cv::Mat dist_coeff = cv::Mat::zeros(5, 1, CV_64FC1);
  dist_coeff.at<double>(0, 0) = params.disto[0];
  dist_coeff.at<double>(1, 0) = params.disto[1];
  dist_coeff.at<double>(2, 0) = params.disto[3];
  dist_coeff.at<double>(3, 0) = params.disto[4];
  dist_coeff.at<double>(4, 0) = params.disto[2];
  return dist_coeff;
}

void get_stereo_camera_matrices(
    const sl::zed::StereoParameters *params,
    cv::Mat *left_camera_matrix,
    cv::Mat *right_camera_matrix)
{
//  left_camera_matrix.create(3, 3, CV_64FC1);
//  right_camera_matrix.create(3, 3, CV_64FC1);
//  cv::Mat left_mat = left_camera_matrix.getMat();
//  cv::Mat right_mat = right_camera_matrix.getMat();
  *left_camera_matrix = get_camera_matrix(params->LeftCam);
  *right_camera_matrix = get_camera_matrix(params->RightCam);
}

void get_stereo_projection_matrices(
    const sl::zed::StereoParameters *params,
    cv::Mat *left_projection_matrix,
    cv::Mat *right_projection_matrix)
{
//  left_camera_matrix.create(3, 3, CV_64FC1);
//  right_camera_matrix.create(3, 3, CV_64FC1);
//  cv::Mat left_mat = left_camera_matrix.getMat();
//  cv::Mat right_mat = right_camera_matrix.getMat();
  *left_projection_matrix = cv::Mat::zeros(3, 4, CV_64FC1);
  *right_projection_matrix = cv::Mat::zeros(3, 4, CV_64FC1);
  get_camera_matrix(params->LeftCam).copyTo((*left_projection_matrix)(cv::Rect(0, 0, 3, 3)));
  get_camera_matrix(params->RightCam).copyTo((*right_projection_matrix)(cv::Rect(0, 0, 3, 3)));
  right_projection_matrix->at<double>(0, 3) = params->RightCam.fx * params->baseline;
}

void get_stereo_distortion_coefficients(
    const sl::zed::StereoParameters *params,
    cv::Mat *left_dist_coeff,
    cv::Mat *right_dist_coeff)
{
//  left_dist_coeff.create(3, 3, CV_64FC1);
//  right_dist_coeff.create(3, 3, CV_64FC1);
//  cv::Mat left_mat = left_dist_coeff.getMat();
//  cv::Mat right_mat = right_dist_coeff.getMat();
  *left_dist_coeff = get_distortion_coefficients(params->LeftCam);
  *right_dist_coeff = get_distortion_coefficients(params->RightCam);
}

cv::Mat draw_keypoints(cv::InputArray img, const std::vector<cv::KeyPoint> &keypoints)
{
  cv::Mat img_with_keypoints;
  cv::drawKeypoints(img, keypoints, img_with_keypoints, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  return img_with_keypoints;
}

cv::Mat draw_feature_matches(cv::InputArray left_img, const std::vector<cv::KeyPoint> &left_keypoints, cv::InputArray right_img, const std::vector<cv::KeyPoint> &right_keypoints, const std::vector<cv::DMatch> &matches)
{
  cv::Mat match_img;
  cv::drawMatches(left_img, left_keypoints, right_img, right_keypoints, matches, match_img);
  return match_img;
}

cv::Mat convert_to_grayscale(cv::InputArray img)
{
  cv::Mat grayscale_img;
  cv::cvtColor(img, grayscale_img, CV_RGB2GRAY);
  return grayscale_img;
}

void sparse_stereo_matching(video::VideoSourceZED &video, const cv::InputArray left_color_img, cv::InputArray right_color_img)
{
  sl::zed::Camera *zed = video.getNativeCamera();
  print_stereo_parameters(zed->getParameters());

  cv::Mat left_img = convert_to_grayscale(left_color_img);
  cv::Mat right_img = convert_to_grayscale(right_color_img);
  std::cout << "channels: " << left_img.channels() << ", dtype: " << left_img.type() << ", continuous: " << left_img.isContinuous() << ", depth: " << left_img.depth() << std::endl;
  std::cout << "width: " << left_img.cols << ", height: " << left_img.rows << ", elementSize: " << left_img.elemSize1() << std::endl;

  const int hessian_threshold = 400;
  auto detector = cv::xfeatures2d::SURF::create(hessian_threshold);
  auto left_keypoints = detect_keypoints(detector, left_img);
  auto right_keypoints = detect_keypoints(detector, right_img);
  auto left_descriptors = compute_feature_descriptors(detector, left_img, left_keypoints);
  auto right_descriptors = compute_feature_descriptors(detector, right_img, right_keypoints);

  std::vector<cv::DMatch> matches = match_features_flann_knn2(left_descriptors, right_descriptors);

  cv::Mat left_camera_matrix;
  cv::Mat right_camera_matrix;
  get_stereo_camera_matrices(zed->getParameters(), &left_camera_matrix, &right_camera_matrix);
  std::cout << "-- camera matrices --" << std::endl;
  std::cout << "left: " << std::endl << left_camera_matrix << std::endl;
  std::cout << "right: " << std::endl << right_camera_matrix << std::endl;

  cv::Mat left_projection_matrix;
  cv::Mat right_projection_matrix;
  get_stereo_projection_matrices(zed->getParameters(), &left_projection_matrix, &right_projection_matrix);
  std::cout << "-- projection matrices --" << std::endl;
  std::cout << "left: " << std::endl << left_projection_matrix << std::endl;
  std::cout << "right: " << std::endl << right_projection_matrix << std::endl;

  cv::Mat left_R;
  cv::Mat left_t;
  cv::Mat right_R;
  cv::Mat right_t;
  get_stereo_RT(zed->getParameters(), left_R, left_t, right_R, right_t);
  std::cout << "-- stereo transformation matrices --" << std::endl;
  std::cout << "left_R: " << std::endl << left_R << std::endl;
  std::cout << "left_t: " << std::endl << left_t << std::endl;
  std::cout << "right_R: " << std::endl << right_R << std::endl;
  std::cout << "right_t: " << std::endl << right_t << std::endl;

  // Note: Make sure to use double matrices as input here, otherwise the resulting matrix will be all zeros!
  cv::Mat essential_matrix;
  cv::sfm::essentialFromRt(left_R, left_t, right_R, right_t, essential_matrix);
  std::cout << "essential_matrix: " << std::endl << essential_matrix << std::endl;

  cv::Mat fundamental_matrix;
  cv::sfm::fundamentalFromEssential(essential_matrix, left_camera_matrix, right_camera_matrix, fundamental_matrix);
  std::cout << "fundamental matrix: " << std::endl << fundamental_matrix << std::endl;

  // TODO: Get proper stereo calibration

  cv::Mat left_dist_coeff;
  cv::Mat right_dist_coeff;
  get_stereo_distortion_coefficients(zed->getParameters(), &left_dist_coeff, &right_dist_coeff);

  std::vector<cv::Point2d> left_points;
  std::vector<cv::Point2d> right_points;
  for (auto &keypoint : left_keypoints)
  {
    left_points.push_back(keypoint.pt);
  }
  for (auto &keypoint : right_keypoints)
  {
    right_points.push_back(keypoint.pt);
  }
  cv::Mat left_undist_points;
  cv::Mat right_undist_points;
  cv::undistortPoints(left_points, left_undist_points, left_camera_matrix, left_dist_coeff, cv::noArray(), left_camera_matrix);
  cv::undistortPoints(right_points, right_undist_points, right_camera_matrix, right_dist_coeff, cv::noArray(), right_camera_matrix);

  // Compute epipolar constraint of matches
  std::cout << "-- epipolar constraints --" << std::endl;
  std::vector<cv::DMatch> best_matches;
  for (int i = 0; i < matches.size(); ++i)
  {
    cv::Mat point1 = cv::Mat(1, 1, CV_64FC2);
    point1.at<cv::Point2d>(0, 0) = left_undist_points.at<cv::Point2d>(matches[i].queryIdx);
    cv::Mat hom_point1;
    cv::convertPointsToHomogeneous(point1, hom_point1);
    cv::Mat point2 = cv::Mat(1, 1, CV_64FC2);
    point2.at<cv::Point2d>(0, 0) = right_undist_points.at<cv::Point2d>(matches[i].trainIdx);
    cv::Mat hom_point2;
    cv::convertPointsToHomogeneous(point2, hom_point2);
    cv::Mat hom_point1_mat(3, 1, CV_64FC1);
    hom_point1_mat.at<double>(0, 0) = hom_point1.at<cv::Point3d>(0, 0).x;
    hom_point1_mat.at<double>(1, 0) = hom_point1.at<cv::Point3d>(0, 0).y;
    hom_point1_mat.at<double>(2, 0) = hom_point1.at<cv::Point3d>(0, 0).z;
    cv::Mat hom_point2_mat(3, 1, CV_64FC1);
    hom_point2_mat.at<double>(0, 0) = hom_point2.at<cv::Point3d>(0, 0).x;
    hom_point2_mat.at<double>(1, 0) = hom_point2.at<cv::Point3d>(0, 0).y;
    hom_point2_mat.at<double>(2, 0) = hom_point2.at<cv::Point3d>(0, 0).z;
//    std::cout << "hom_point1_mat: " << hom_point1_mat << std::endl;
//    std::cout << "hom_point2_mat: " << hom_point2_mat << std::endl;
    cv::Mat epipolar_constraint_mat = hom_point2_mat.t() * fundamental_matrix * hom_point1_mat;
    CV_Assert(epipolar_constraint_mat.rows == 1 && epipolar_constraint_mat.cols == 1);

    double epipolar_constraint = epipolar_constraint_mat.at<double>(0, 0);
    if (std::abs(epipolar_constraint) < 0.005)
    {
      best_matches.push_back(matches[i]);
    }
//    std::cout << i << ": " << epipolar_constraint << std::endl;
  }
  std::cout << "Keeping " << best_matches.size() << " from " << matches.size() << std::endl;

  std::vector<cv::Point2d> left_correct_points;
  std::vector<cv::Point2d> right_correct_points;
  cv::correctMatches(fundamental_matrix, left_undist_points, right_undist_points, left_correct_points, right_correct_points);
//  for (int i = 0; i < matches.size(); ++i)
//  {
//    left_keypoints[matches[i].queryIdx].pt = new_left_points[i];
//    right_keypoints[matches[i].trainIdx].pt = new_right_points[i];
//  }

  std::vector<cv::Point2f> left_best_points;
  std::vector<cv::Point2f> right_best_points;
  for (int i = 0; i < best_matches.size(); ++i)
  {
    left_best_points.push_back(left_correct_points[best_matches[i].queryIdx]);
    right_best_points.push_back(left_correct_points[best_matches[i].queryIdx]);
  }
  cv::Mat keypoints_4d(4, best_matches.size(), CV_64FC1);
  cv::triangulatePoints(left_projection_matrix, right_projection_matrix, left_best_points, right_best_points, keypoints_4d);
  std::cout << "-- keypoints --" << std::endl;
  for (int i = 0; i < best_matches.size(); ++i)
  {
    std::cout << i << ": " << keypoints_4d.col(i) << std::endl;
  }

  auto left_img_with_keypoints = draw_keypoints(left_img, left_keypoints);
  auto right_img_with_keypoints = draw_keypoints(right_img, right_keypoints);
  cv::imshow("Left keypoints", left_img_with_keypoints);
  cv::imshow("Right keypoints", right_img_with_keypoints);

  auto match_img = draw_feature_matches(left_img, left_keypoints, right_img, right_keypoints, best_matches);
  cv::imshow("Keypoint matches", match_img);

  cv::waitKey(0);
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
    TCLAP::ValueArg<bool> show_arg("s", "show", "Show captured video", false, true, "boolean", cmd);
    TCLAP::ValueArg<std::string> zed_params_arg("", "zed-params", "ZED parameter file", false, "", "filename", cmd);

    cmd.parse(argc, argv);

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

      sparse_stereo_matching(video, disp_left, disp_right);

      break;

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
