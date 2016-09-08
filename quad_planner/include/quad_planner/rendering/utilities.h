//==================================================
// utilities.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 22, 2016
//==================================================

#pragma once

#include <quad_planner/rendering/glm.h>
#include <Eigen/Dense>

namespace quad_planner
{
namespace rendering
{

class Utilities
{
public:
  Utilities() = delete;
  Utilities(const Utilities &) = delete;
  Utilities& operator=(const Utilities&) = delete;
//  virtual ~Utilities();

  static Eigen::Vector3d glmToEigen(const glm::vec3 &vec);
  static glm::vec3 eigenToGlm(const Eigen::Vector3d &vec);

  static Eigen::MatrixXd glmToEigenMatrixXd(const glm::mat4 &mat);
  static Eigen::MatrixXf glmToEigenMatrixXf(const glm::mat4 &mat);
  static glm::mat4 eigenToGlm(const Eigen::MatrixXd &mat);
  static glm::mat4 eigenToGlm(const Eigen::MatrixXf &mat);
  static Eigen::Matrix<double, 4, 4> glmToEigenMatrix44d(const glm::mat4 &mat);
  static Eigen::Matrix<float, 4, 4> glmToEigenMatrix44f(const glm::mat4 &mat);
  static glm::mat4 eigenToGlm(const Eigen::Matrix<double, 4, 4> &mat);
  static glm::mat4 eigenToGlm(const Eigen::Matrix<float, 4, 4> &mat);

  template <typename _Scalar, int _Rows>
  static void convertVectorToMatrix(const std::vector<Eigen::Matrix<_Scalar, _Rows, 1>> &vector,
                                    Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> *matrix_ptr)
  {
    *matrix_ptr = Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic>::Zero(_Rows, vector.size());
    for (int i = 0; i < vector.size(); ++i)
    {
      matrix_ptr->col(i) = vector[i];
    }
  }

  template <typename _Scalar, int _Rows>
  static void convertMatrixToVector(const Eigen::Matrix<_Scalar, Eigen::Dynamic, Eigen::Dynamic> &matrix,
                                    std::vector<Eigen::Matrix<_Scalar, _Rows, 1>> *vector_ptr)
  {
    *vector_ptr = std::vector<Eigen::Matrix<_Scalar, _Rows, 1>>(matrix.cols());
    for (int i = 0; i < matrix.cols(); ++i)
    {
      (*vector_ptr)[i] = matrix.col(i);
    }
  }
};

} /* namespace rendering */
} /* namespace quad_planner */
