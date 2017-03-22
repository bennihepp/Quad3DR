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
#include "../eigen.h"
#include "mLib.h"
#include "../mesh/triangle_mesh.h"

namespace bh {

class MLibUtilities {
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

  template <typename FloatT>
  static Eigen::Matrix<FloatT, 2, 2> convertMlibToEigen(const ml::Matrix2x2<FloatT>& ml_matrix) {
    Eigen::Matrix<FloatT, 2, 2> matrix;
    for (std::size_t row = 0; row < static_cast<std::size_t>(matrix.rows()); ++row) {
      for (std::size_t col = 0; col < static_cast<std::size_t>(matrix.cols()); ++col) {
        matrix(row, col) = ml_matrix(row, col);
      }
    }
    return matrix;
  }

  template <typename FloatT>
  static Eigen::Matrix<FloatT, 3, 3> convertMlibToEigen(const ml::Matrix3x3<FloatT>& ml_matrix) {
    Eigen::Matrix<FloatT, 3, 3> matrix;
    for (std::size_t row = 0; row < static_cast<std::size_t>(matrix.rows()); ++row) {
      for (std::size_t col = 0; col < static_cast<std::size_t>(matrix.cols()); ++col) {
        matrix(row, col) = ml_matrix(row, col);
      }
    }
    return matrix;
  }

    template <typename FloatT>
    static Eigen::Matrix<FloatT, 4, 4> convertMlibToEigen(const ml::Matrix4x4<FloatT>& ml_matrix) {
      Eigen::Matrix<FloatT, 4, 4> matrix;
      for (std::size_t row = 0; row < static_cast<std::size_t>(matrix.rows()); ++row) {
        for (std::size_t col = 0; col < static_cast<std::size_t>(matrix.cols()); ++col) {
          matrix(row, col) = ml_matrix(row, col);
        }
      }
      return matrix;
    }

    template <typename FloatT>
    static ml::Matrix2x2<FloatT> convertEigenToMlib(const Eigen::Matrix<FloatT, 2, 2>& eigen_matrix) {
      ml::Matrix2x2<FloatT> matrix;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_matrix.rows()); ++row) {
        for (std::size_t col = 0; col < static_cast<std::size_t>(eigen_matrix.cols()); ++col) {
          matrix(row, col) = eigen_matrix(row, col);
        }
      }
      return matrix;
    }

    template <typename FloatT>
    static ml::Matrix3x3<FloatT> convertEigenToMlib(const Eigen::Matrix<FloatT, 3, 3>& eigen_matrix) {
      ml::Matrix3x3<FloatT> matrix;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_matrix.rows()); ++row) {
        for (std::size_t col = 0; col < static_cast<std::size_t>(eigen_matrix.cols()); ++col) {
          matrix(row, col) = eigen_matrix(row, col);
        }
      }
      return matrix;
    }

    template <typename FloatT>
    static ml::Matrix4x4<FloatT> convertEigenToMlib(const Eigen::Matrix<FloatT, 4, 4>& eigen_matrix) {
      ml::Matrix4x4<FloatT> matrix;
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

    template <typename FloatT>
    static Eigen::Matrix<FloatT, 2, 1> convertMlibToEigen(const ml::vec2<FloatT>& ml_vector) {
      Eigen::Matrix<FloatT, 2, 1> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(vector.rows()); ++row) {
        vector(row) = ml_vector[row];
      }
      return vector;
    }

    template <typename FloatT>
    static Eigen::Matrix<FloatT, 3, 1> convertMlibToEigen(const ml::vec3<FloatT>& ml_vector) {
      Eigen::Matrix<FloatT, 3, 1> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(vector.rows()); ++row) {
        vector(row) = ml_vector[row];
      }
      return vector;
    }

    template <typename FloatT>
    static Eigen::Matrix<FloatT, 4, 1> convertMlibToEigen(const ml::vec4<FloatT>& ml_vector) {
      Eigen::Matrix<FloatT, 4, 1> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(vector.rows()); ++row) {
        vector(row) = ml_vector[row];
      }
      return vector;
    }

    template <typename FloatT>
    static ml::vec2<FloatT> convertEigenToMlib(const Eigen::Matrix<FloatT, 2, 1>& eigen_vector) {
      ml::vec2<FloatT> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_vector.rows()); ++row) {
        vector[row] = eigen_vector(row);
      }
      return vector;
    }

    template <typename FloatT>
    static ml::vec3<FloatT> convertEigenToMlib(const Eigen::Matrix<FloatT, 3, 1>& eigen_vector) {
      ml::vec3<FloatT> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_vector.rows()); ++row) {
        vector[row] = eigen_vector(row);
      }
      return vector;
    }

    template <typename FloatT>
    static ml::vec4<FloatT> convertEigenToMlib(const Eigen::Matrix<FloatT, 4, 1>& eigen_vector) {
      ml::vec4<FloatT> vector;
      for (std::size_t row = 0; row < static_cast<std::size_t>(eigen_vector.rows()); ++row) {
        vector[row] = eigen_vector(row);
      }
      return vector;
    }

    //
    // Mesh conversions between bh and mLib
    //

    template <typename FloatT>
    static TriangleMesh<FloatT> convertMlibToBh(const ml::MeshData<FloatT>& ml_mesh) {
      TriangleMesh<FloatT> mesh;
      mesh.triangleVertexIndices().reserve(ml_mesh.m_FaceIndicesVertices.size());
      for (size_t i = 0; i < ml_mesh.m_FaceIndicesVertices.size(); ++i) {
        const typename ml::MeshData<FloatT>::Indices::Face& face = ml_mesh.m_FaceIndicesVertices[i];
        mesh.triangleVertexIndices().emplace_back(face[0], face[1], face[2]);
      }
      std::transform(std::begin(ml_mesh.m_Vertices), std::end(ml_mesh.m_Vertices), std::back_inserter(mesh.vertices()),
                     [] (const ml::vec3<FloatT>& ml_vec) {
        return convertMlibToEigen(ml_vec);
      });
      if (ml_mesh.hasNormals() && !ml_mesh.hasNormalIndices()) {
        std::transform(std::begin(ml_mesh.m_Normals), std::end(ml_mesh.m_Normals), std::back_inserter(mesh.normals()),
                       [] (const ml::vec3<FloatT>& ml_vec) {
          return convertMlibToEigen(ml_vec);
        });
      }
      if (ml_mesh.hasColors() && !ml_mesh.hasColorIndices()) {
        std::transform(std::begin(ml_mesh.m_Colors), std::end(ml_mesh.m_Colors), std::back_inserter(mesh.colors()),
                       [] (const ml::vec4<FloatT>& ml_vec) {
          return convertMlibToEigen(ml_vec);
        });
      }
      if (ml_mesh.hasTexCoords() && !ml_mesh.hasTexCoordsIndices()) {
        std::transform(std::begin(ml_mesh.m_TextureCoords), std::end(ml_mesh.m_TextureCoords), std::back_inserter(mesh.textureUVs()),
                       [] (const ml::vec2<FloatT>& ml_vec) {
          return convertMlibToEigen(ml_vec);
        });
      }
      return mesh;
    }

    template <typename FloatT>
    static ml::MeshData<FloatT> convertBhToMlib(const TriangleMesh<FloatT>& mesh) {
      ml::MeshData<FloatT> ml_mesh;
      ml_mesh.m_FaceIndicesVertices.resize(mesh.triangleVertexIndices().size(), 3);
      for (auto it = std::begin(mesh.triangleVertexIndices()); it != std::end(mesh.triangleVertexIndices()); ++it) {
        const typename TriangleMesh<FloatT>::Vector3i& indices = *it;
        const size_t idx = it - std::begin(mesh.triangleVertexIndices());
        ml_mesh.m_FaceIndicesVertices[idx][0] = indices(0);
        ml_mesh.m_FaceIndicesVertices[idx][1] = indices(1);
        ml_mesh.m_FaceIndicesVertices[idx][2] = indices(2);
      }
      std::transform(std::begin(mesh.vertices()), std::end(mesh.vertices()), std::back_inserter(ml_mesh.m_Vertices),
                     [] (const bh::Vector3<FloatT>& vec) {
        return convertEigenToMlib(vec);
      });
      if (mesh.hasNormals()) {
        std::transform(std::begin(mesh.normals()), std::end(mesh.normals()), std::back_inserter(ml_mesh.m_Normals),
                       [] (const bh::Vector3<FloatT>& vec) {
          return convertEigenToMlib(vec);
        });
      }
      if (mesh.hasColors()) {
        std::transform(std::begin(mesh.colors()), std::end(mesh.colors()), std::back_inserter(ml_mesh.m_Colors),
                       [] (const bh::Vector4<FloatT>& vec) {
          return convertEigenToMlib(vec);
        });
      }
      if (mesh.hasTextureUVs()) {
        std::transform(std::begin(mesh.textureUVs()), std::end(mesh.textureUVs()), std::back_inserter(ml_mesh.m_TextureCoords),
                       [] (const bh::Vector2<FloatT>& vec) {
          return convertEigenToMlib(vec);
        });
      }
      return ml_mesh;
    }
};

} /* namespace bh */
