//==================================================
// mLibUtils.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 8, 2016
//==================================================
#pragma once

#include <vector>
#include <utility>
#include <cstdint>
#include <ait/eigen.h>
#include <ait/mLib.h>

namespace ait
{

class MLibUtilities
{
public:
  template <typename T>
  static void savePointCloudToOff(const std::string &filename, const ml::PointCloud<T> &pc, bool save_color=true)
  {
    std::ofstream out(filename);
    if (!out.is_open())
    {
      throw std::runtime_error("Unable to open output file: " + filename);
    }
    out << "COFF" << std::endl;
    out << pc.m_points.size() << " " << 0 << " " << 0 << std::endl;
    for (int i = 0; i < pc.m_points.size(); ++i)
    {
      const ml::vec3<T> &p = pc.m_points[i];
      out << p.x << " " << p.y << " " << p.z;
      if (save_color)
      {
        const ml::vec4<T> &c = pc.m_colors[i];
        out << " " << c.r << " " << c.g << " " << c.b << " " << c.w;
      }
      else
      {
        ml::vec4<T> c(1, 1, 1, 1);
        out << " " << c.r << " " << c.g << " " << c.b << " " << c.w;
      }
      out << std::endl;
    }
    out.close();
  }

  //
  // Matrix conversions between Eigen and mLib
  //

  template <typename FloatType>
  static Eigen::Matrix<FloatType, 2, 2> convertMlibToEigen(const ml::Matrix2x2<FloatType>& ml_matrix) {
    Eigen::Matrix<FloatType, 2, 2> matrix;
    for (std::size_t row = 0; row < static_cast<std::size_t>(matrix.rows()); ++row) {
      for (std::size_t col = 0; col < static_cast<std::size_t>(matrix.cols()); ++col) {
        matrix(row, col) = ml_matrix(row, col);
      }
    }
    return matrix;
  }

  template <typename FloatType>
  static Eigen::Matrix<FloatType, 3, 3> convertMlibToEigen(const ml::Matrix3x3<FloatType>& ml_matrix) {
    Eigen::Matrix<FloatType, 3, 3> matrix;
    for (std::size_t row = 0; row < static_cast<std::size_t>(matrix.rows()); ++row) {
      for (std::size_t col = 0; col < static_cast<std::size_t>(matrix.cols()); ++col) {
        matrix(row, col) = ml_matrix(row, col);
      }
    }
    return matrix;
  }

    template <typename FloatType>
    static Eigen::Matrix<FloatType, 4, 4> convertMlibToEigen(const ml::Matrix4x4<FloatType>& ml_matrix) {
      Eigen::Matrix<FloatType, 4, 4> matrix;
      for (std::size_t row = 0; row < static_cast<std::size_t>(matrix.rows()); ++row) {
        for (std::size_t col = 0; col < static_cast<std::size_t>(matrix.cols()); ++col) {
          matrix(row, col) = ml_matrix(row, col);
        }
      }
      return matrix;
    }

    template <typename FloatType>
    static ml::Matrix2x2<FloatType> convertEigenToMlib(const Eigen::Matrix<FloatType, 2, 2>& eigen_matrix) {
      ml::Matrix2x2<FloatType> matrix;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_matrix.rows()); ++row) {
        for (std::size_t col = 0; col < static_cast<std::size_t>(eigen_matrix.cols()); ++col) {
          matrix(row, col) = eigen_matrix(row, col);
        }
      }
      return matrix;
    }

    template <typename FloatType>
    static ml::Matrix3x3<FloatType> convertEigenToMlib(const Eigen::Matrix<FloatType, 3, 3>& eigen_matrix) {
      ml::Matrix3x3<FloatType> matrix;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_matrix.rows()); ++row) {
        for (std::size_t col = 0; col < static_cast<std::size_t>(eigen_matrix.cols()); ++col) {
          matrix(row, col) = eigen_matrix(row, col);
        }
      }
      return matrix;
    }

    template <typename FloatType>
    static ml::Matrix4x4<FloatType> convertEigenToMlib(const Eigen::Matrix<FloatType, 4, 4>& eigen_matrix) {
      ml::Matrix4x4<FloatType> matrix;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_matrix.rows()); ++row) {
        for (std::size_t col = 0; col < static_cast<std::size_t>(eigen_matrix.cols()); ++col) {
          matrix(row, col) = eigen_matrix(row, col);
        }
      }
      return matrix;
    }

    //
    // Vector conversions between Eigen and mLib
    //

    template <typename FloatType>
    static Eigen::Matrix<FloatType, 2, 1> convertEigenToMlib(const ml::vec2<FloatType>& ml_vector) {
      Eigen::Matrix<FloatType, 2, 1> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(vector.rows()); ++row) {
        vector(row) = ml_vector[row];
      }
      return vector;
    }

    template <typename FloatType>
    static Eigen::Matrix<FloatType, 3, 1> convertEigenToMlib(const ml::vec3<FloatType>& ml_vector) {
      Eigen::Matrix<FloatType, 3, 1> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(vector.rows()); ++row) {
        vector(row) = ml_vector[row];
      }
      return vector;
    }

    template <typename FloatType>
    static Eigen::Matrix<FloatType, 4, 1> convertEigenToMlib(const ml::vec4<FloatType>& ml_vector) {
      Eigen::Matrix<FloatType, 4, 1> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(vector.rows()); ++row) {
        vector(row) = ml_vector[row];
      }
      return vector;
    }

    template <typename FloatType>
    static ml::vec2<FloatType> convertEigenToMlib(const Eigen::Matrix<FloatType, 2, 1>& eigen_vector) {
      ml::vec2<FloatType> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_vector.rows()); ++row) {
        vector[row] = eigen_vector(row);
      }
      return vector;
    }

    template <typename FloatType>
    static ml::vec3<FloatType> convertEigenToMlib(const Eigen::Matrix<FloatType, 3, 1>& eigen_vector) {
      ml::vec3<FloatType> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_vector.rows()); ++row) {
        vector[row] = eigen_vector(row);
      }
      return vector;
    }

    template <typename FloatType>
    static ml::vec4<FloatType> convertEigenToMlib(const Eigen::Matrix<FloatType, 4, 1>& eigen_vector) {
      ml::vec4<FloatType> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_vector.rows()); ++row) {
        vector[row] = eigen_vector(row);
      }
      return vector;
    }

};

} /* namespace stereo */
