//==================================================
// utilities.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 22, 2016
//==================================================

#include <quad_planner/rendering/utilities.h>

namespace quad_planner
{
namespace rendering
{

Eigen::Vector3d Utilities::glmToEigen(const glm::vec3 &vec)
{
  return Eigen::Vector3d(vec.x, vec.y, vec.z);
}
glm::vec3 Utilities::eigenToGlm(const Eigen::Vector3d &vec)
{
  return glm::vec3(vec(0), vec(1), vec(2));
}

Eigen::MatrixXd Utilities::glmToEigenMatrixXd(const glm::mat4 &mat)
{
  Eigen::MatrixXd mat_ret(4, 4);
  for (int col = 0; col < 4; ++col)
  {
    for (int row = 0; row < 4; ++row)
    {
      mat_ret(row, col) = mat[col][row];
    }
  }
  return mat_ret;
}

Eigen::MatrixXf Utilities::glmToEigenMatrixXf(const glm::mat4 &mat)
{
  Eigen::MatrixXf mat_ret(4, 4);
  for (int col = 0; col < 4; ++col)
  {
    for (int row = 0; row < 4; ++row)
    {
      mat_ret(row, col) = mat[col][row];
    }
  }
  return mat_ret;
}

glm::mat4 Utilities::eigenToGlm(const Eigen::MatrixXd &mat)
{
  if (mat.rows() != 4 || mat.cols() != 4)
  {
    throw std::runtime_error("eigenToGlm() failed: Matrix size has to be 4x4");
  }
  glm::mat4 mat_ret;
  for (int col = 0; col < 4; ++col)
  {
    for (int row = 0; row < 4; ++row)
    {
      mat_ret[col][row] = mat(row, col);
    }
  }
  return mat_ret;
}

glm::mat4 Utilities::eigenToGlm(const Eigen::MatrixXf &mat)
{
  if (mat.rows() != 4 || mat.cols() != 4)
  {
    throw std::runtime_error("eigenToGlm() failed: Matrix size has to be 4x4");
  }
  glm::mat4 mat_ret;
  for (int col = 0; col < 4; ++col)
  {
    for (int row = 0; row < 4; ++row)
    {
      mat_ret[col][row] = mat(row, col);
    }
  }
  return mat_ret;
}

Eigen::Matrix<double, 4, 4> Utilities::glmToEigenMatrix44d(const glm::mat4 &mat)
{
  Eigen::Matrix<double, 4, 4> mat_ret;
  for (int col = 0; col < 4; ++col)
  {
    for (int row = 0; row < 4; ++row)
    {
      mat_ret(row, col) = mat[col][row];
    }
  }
  return mat_ret;
}

Eigen::Matrix<float, 4, 4> Utilities::glmToEigenMatrix44f(const glm::mat4 &mat)
{
  Eigen::Matrix<float, 4, 4> mat_ret;
  for (int col = 0; col < 4; ++col)
  {
    for (int row = 0; row < 4; ++row)
    {
      mat_ret(row, col) = mat[col][row];
    }
  }
  return mat_ret;
}

glm::mat4 Utilities::eigenToGlm(const Eigen::Matrix<double, 4, 4> &mat)
{
  glm::mat4 mat_ret;
  for (int col = 0; col < 4; ++col)
  {
    for (int row = 0; row < 4; ++row)
    {
      mat_ret[col][row] = mat(row, col);
    }
  }
  return mat_ret;
}

glm::mat4 Utilities::eigenToGlm(const Eigen::Matrix<float, 4, 4> &mat)
{
  glm::mat4 mat_ret;
  for (int col = 0; col < 4; ++col)
  {
    for (int row = 0; row < 4; ++row)
    {
      mat_ret[col][row] = mat(row, col);
    }
  }
  return mat_ret;
}

} /* namespace rendering */
} /* namespace quad_planner */
