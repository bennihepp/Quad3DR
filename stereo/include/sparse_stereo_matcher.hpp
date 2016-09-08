//==================================================
// sparse_stereo_matcher.hpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 3, 2016
//==================================================

#include <utilities.h>
#include <limits>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/sfm.hpp>

namespace stereo
{

template <typename T>
SparseStereoMatcher<T>::SparseStereoMatcher(const StereoCameraCalibration &calib)
: calib_(calib)
{

}

template <typename T>
SparseStereoMatcher<T>::~SparseStereoMatcher()
{
}


template <typename T>
cv::Mat SparseStereoMatcher<T>::undistortPoints(cv::InputArray &points, cv::InputArray camera_matrix, cv::InputArray dist_coefficients) const
{
  cv::InputArray R = cv::noArray();
  cv::InputArray P = camera_matrix;
  cv::Mat undistorted_points;
  cv::undistortPoints(points, undistorted_points, camera_matrix, dist_coefficients, R, P);
  return undistorted_points;
}

template <typename T>
std::vector<cv::KeyPoint> SparseStereoMatcher<T>::detectFeatureKeypoints(const cv::InputArray img, std::vector<cv::Point2d> *points_ptr) const
{
  std::vector<cv::KeyPoint> keypoints;
  detector_->detect(img, keypoints);
  if (points_ptr != nullptr)
  {
    for (auto &keypoint : keypoints)
    {
      points_ptr->push_back(keypoint.pt);
    }
  }
  return keypoints;
}

template <typename T>
cv::Mat SparseStereoMatcher<T>::computeFeatureDescriptors(cv::InputArray img, std::vector<cv::KeyPoint> &keypoints) const
{
  cv::Mat descriptors;
  detector_->compute(img, keypoints, descriptors);
  return descriptors;
}

template <typename T>
std::vector<cv::DMatch> SparseStereoMatcher<T>::filterMatches(const std::vector<cv::DMatch> &all_matches, double good_threshold_multiplier, double min_good_threshold) const
{
  double max_dist = -std::numeric_limits<double>::infinity();
  double min_dist = std::numeric_limits<double>::infinity();
  // Quick calculation of max and min distances between keypoints
  for (int i = 0; i < all_matches.size(); i++)
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
//  std::cout << "-- Max dist: " << max_dist << ", Min dist: " << min_dist << std::endl;
  // Keep only "good" matches (i.e. whose distance is less than 2*min_dist,
  // or a small arbitary value in case that min_dist is very small).
  std::vector<cv::DMatch> good_matches;
  double good_threshold = std::max(good_threshold_multiplier * min_dist, min_good_threshold);
  for (int i = 0; i < all_matches.size(); i++)
  {
    if (all_matches[i].distance <= good_threshold)
    {
      good_matches.push_back(all_matches[i]);
    }
  }
  return good_matches;
}

template <typename T>
std::vector<cv::DMatch> SparseStereoMatcher<T>::matchFeaturesBf(cv::InputArray left_descriptors, cv::InputArray right_descriptors) const
{
//   Brute-Force matching
  cv::BFMatcher matcher(cv::NORM_L2);
  std::vector<cv::DMatch> matches;
  matcher.match(left_descriptors, right_descriptors, matches);
  return filterMatches(matches);
}

template <typename T>
std::vector<cv::DMatch> SparseStereoMatcher<T>::matchFeaturesFlann(cv::InputArray left_descriptors, cv::InputArray right_descriptors) const
{
  // FLANN matching
  auto index_params = cv::makePtr<cv::flann::KDTreeIndexParams>(5);
  auto search_param = cv::makePtr<cv::flann::SearchParams>(50, 0, true);
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> all_matches;
  matcher.match(left_descriptors, right_descriptors, all_matches);
  return filterMatches(all_matches);
}

template <typename T>
std::vector<cv::DMatch> SparseStereoMatcher<T>::matchFeaturesFlannKnn2(cv::InputArray left_descriptors, cv::InputArray right_descriptors, double ratio_test_threshold) const
{
  // FLANN matching with kNN (k=2)
  int k = 2;
  auto index_params = cv::makePtr<cv::flann::KDTreeIndexParams>(5);
  auto search_param = cv::makePtr<cv::flann::SearchParams>(50, 0, true);
  cv::FlannBasedMatcher matcher;
  std::vector<std::vector<cv::DMatch>> all_matches;
  matcher.knnMatch(left_descriptors, right_descriptors, all_matches, k);

  // Lowe's ratio test
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

template <typename T>
SparseMatchResult SparseStereoMatcher<T>::match(const cv::InputArray left_color_img, cv::InputArray right_color_img) const
{
  SparseMatchResult match_result;

  cv::Mat left_img = Utilities::convertToGrayscale(left_color_img);
  cv::Mat right_img = Utilities::convertToGrayscale(right_color_img);
  std::cout << "channels: " << left_img.channels() << ", dtype: " << left_img.type() << ", continuous: " << left_img.isContinuous() << ", depth: " << left_img.depth() << std::endl;
  std::cout << "width: " << left_img.cols << ", height: " << left_img.rows << ", elementSize: " << left_img.elemSize1() << std::endl;

  std::vector<cv::Point2d> left_points;
  std::vector<cv::Point2d> right_points;
  auto left_keypoints = detectFeatureKeypoints(left_img, &left_points);
  auto right_keypoints = detectFeatureKeypoints(right_img, &right_points);
  auto left_descriptors = computeFeatureDescriptors(left_img, left_keypoints);
  auto right_descriptors = computeFeatureDescriptors(right_img, right_keypoints);

  std::vector<cv::DMatch> matches = matchFeaturesFlannKnn2(left_descriptors, right_descriptors);

  cv::Mat left_undist_points = undistortPoints(left_points, calib_.left.camera_matrix, calib_.left.dist_coeffs);
  cv::Mat right_undist_points = undistortPoints(right_points, calib_.right.camera_matrix, calib_.right.dist_coeffs);

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
    cv::Mat epipolar_constraint_mat = hom_point2_mat.t() * calib_.fundamental_matrix * hom_point1_mat;
    CV_Assert(epipolar_constraint_mat.rows == 1 && epipolar_constraint_mat.cols == 1);

    double epipolar_constraint = epipolar_constraint_mat.at<double>(0, 0);
    if (std::abs(epipolar_constraint) < 0.005)
    {
      best_matches.push_back(matches[i]);
      match_result.epipolar_constraints.push_back(epipolar_constraint);
    }
//    std::cout << i << ": " << epipolar_constraint << std::endl;
  }
  std::cout << "Keeping " << best_matches.size() << " from " << matches.size() << std::endl;

  std::vector<cv::Point2d> left_correct_points;
  std::vector<cv::Point2d> right_correct_points;
  cv::correctMatches(calib_.fundamental_matrix, left_undist_points, right_undist_points, left_correct_points, right_correct_points);
//  for (int i = 0; i < matches.size(); ++i)
//  {
//    left_keypoints[matches[i].queryIdx].pt = new_left_points[i];
//    right_keypoints[matches[i].trainIdx].pt = new_right_points[i];
//  }

  std::vector<cv::Point2d> left_best_points;
  std::vector<cv::Point2d> right_best_points;
  for (int i = 0; i < best_matches.size(); ++i)
  {
    left_best_points.push_back(left_correct_points[best_matches[i].queryIdx]);
    right_best_points.push_back(left_correct_points[best_matches[i].queryIdx]);
  }
  cv::Mat points_4d(4, best_matches.size(), CV_64FC1);
  cv::triangulatePoints(calib_.projection_matrix_left, calib_.projection_matrix_right, left_best_points, right_best_points, points_4d);
  std::cout << "-- keypoints --" << std::endl;
  for (int i = 0; i < best_matches.size(); ++i)
  {
    std::cout << i << ": " << points_4d.col(i) << std::endl;
  }

  //match_result.points_3d;
  match_result.left_points = std::move(left_best_points);
  match_result.right_points = std::move(right_best_points);
  //match_result.left_keypoints = std::move(left_best_keypoints);
  //match_result.right_keypoints = std::move(right_best_keypoints);
  match_result.left_descriptors = std::move(left_descriptors);
  match_result.right_descriptors = std::move(right_descriptors);
  match_result.matches = std::move(matches);
  //match_result.epipolar_constraints;
  return match_result;
}

} /* namespace stereo */
