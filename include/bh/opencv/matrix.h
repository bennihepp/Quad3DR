//==================================================
// matrix.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 06.04.17
//==================================================
#pragma once

#include "../common.h"
#include "../eigen.h"
#include <opencv2/core.hpp>

namespace bh {
namespace opencv {

template <typename EigenType>
EigenType convertCvToEigen(const cv::Mat& cv_mat) {
  EigenType eigen_mat(cv_mat.rows, cv_mat.cols);
  if (cv_mat.type() == CV_32F) {
    for (size_t row = 0; row < (size_t) cv_mat.rows; ++row) {
      for (size_t col = 0; col < (size_t) cv_mat.cols; ++col) {
        eigen_mat(row, col) = cv_mat.at<float>(row, col);
      }
    }
  }
  else if (cv_mat.type() == CV_64F) {
    for (size_t row = 0; row < (size_t) cv_mat.rows; ++row) {
      for (size_t col = 0; col < (size_t) cv_mat.cols; ++col) {
        eigen_mat(row, col) = cv_mat.at<double>(row, col);
      }
    }
  }
  else {
    throw bh::Error("OpenCV to Eigen conversion only supports floating types");
  }
  return eigen_mat;
}

template <typename Derived>
cv::Mat convertEigenToCv32F(const Derived& eigen_mat) {
  cv::Mat cv_mat(eigen_mat.rows(), eigen_mat.cols(), CV_32F);
  for (size_t row = 0; row < (size_t) eigen_mat.rows(); ++row) {
    for (size_t col = 0; col < (size_t) eigen_mat.cols(); ++col) {
      cv_mat.at<float>(row, col) = eigen_mat(row, col);
    }
  }
  return cv_mat;
}

template <typename Derived>
cv::Mat convertEigenToCv64F(const Derived& eigen_mat) {
  cv::Mat cv_mat(eigen_mat.rows(), eigen_mat.cols(), CV_64F);
  for (size_t row = 0; row < (size_t) eigen_mat.rows(); ++row) {
    for (size_t col = 0; col < (size_t) eigen_mat.cols(); ++col) {
      cv_mat.at<double>(row, col) = eigen_mat(row, col);
    }
  }
  return cv_mat;
}

}
}