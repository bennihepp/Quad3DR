//==================================================
// cuda_math.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 17, 2017
//==================================================

#pragma once

#include <bh/cuda_utils.h>
#include <cstdint>
#include <cfloat>

namespace bh {

template<typename FloatT, std::size_t _Rows>
class CudaVector {
public:
  using FloatType = FloatT;
  static const std::size_t Rows = _Rows;

  __host__ __device__
  static CudaVector Zero() {
    CudaVector vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = 0;
    }
    return vec;
  }

  __host__ __device__
  std::size_t size() const {
    return Rows;
  }

  __host__ __device__
  const FloatT *data() const {
    return vector_;
  }

  __host__ __device__
  FloatT *data() {
    return vector_;
  }

  __host__ __device__
  const FloatT &operator()(const std::size_t index) const {
    return vector_[index];
  }

  __host__ __device__
  FloatT &operator()(const std::size_t index) {
    return vector_[index];
  }

  __host__ __device__
  void copyFrom(const FloatT *data) {
    for (std::size_t i = 0; i < Rows; ++i) {
      vector_[i] = data[i];
    }
  }

  __host__ __device__
  void copyTo(FloatT *data) const {
    for (std::size_t i = 0; i < Rows; ++i) {
      data[i] = vector_[i];
    }
  }

  __host__ __device__
  CudaVector cwiseMin(const CudaVector &other) const {
    CudaVector vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = std::min(vector_[i], other(i));
    }
    return vec;
  }

  __host__ __device__
  CudaVector cwiseMax(const CudaVector &other) const {
    CudaVector vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = std::max(vector_[i], other(i));
    }
    return vec;
  }

  __host__ __device__
  CudaVector cwiseInverse() const {
    CudaVector vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = (FloatT) 1 / vector_[i];
    }
    return vec;
  }

  __host__ __device__
  CudaVector operator+(const CudaVector &other) const {
    CudaVector vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = vector_[i] + other(i);
    }
    return vec;
  }

  __host__ __device__
  CudaVector operator-(const CudaVector &other) const {
    CudaVector vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = vector_[i] - other(i);
    }
    return vec;
  }

  __host__ __device__
  CudaVector operator*(const FloatT &scalar) const {
    CudaVector vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = vector_[i] * scalar;
    }
    return vec;
  }

  __host__ __device__
  FloatT maxCoeff() const {
    FloatT max = vector_[0];
    for (std::size_t i = 1; i < Rows; ++i) {
      max = std::max(max, vector_[i]);
    }
    return max;
  }

  __host__ __device__
  FloatT squaredNorm() const {
    FloatT squared_norm = 0;
    for (std::size_t i = 0; i < Rows; ++i) {
      squared_norm += vector_[i] * vector_[i];
    }
    return squared_norm;
  }

  __host__ __device__
  void print() const {
    for (std::size_t row = 0; row < Rows; ++row) {
      if (row == 0) {
        printf("%f", (*this)(row));
      }
      else {
        printf(" %f", (*this)(row));
      }
    }
    printf("\n");
  }

private:
  FloatT vector_[Rows];
};

template<typename FloatT, std::size_t _Rows, std::size_t _Cols>
class CudaMatrix {
public:
  enum StorageOrder {
    RowMajor,
    ColumnMajor,
  };

  using FloatType = FloatT;
  static const std::size_t Rows = _Rows;
  static const std::size_t Cols = _Cols;

  __host__ __device__
  static CudaMatrix Zero() {
    CudaMatrix mat;
    for (std::size_t i = 0; i < Rows; ++i) {
      for (std::size_t j = 0; j < Cols; ++j) {
        mat(i, j) = 0;
      }
    }
    return mat;
  }

  __host__ __device__
  std::size_t size() const {
    return Rows * Cols;
  }

  __host__ __device__
  const FloatT *data() const {
    return matrix_;
  }

  __host__ __device__
  FloatT *data() {
    return matrix_;
  }

  __host__ __device__
  CudaVector<FloatT, Rows> col(const std::size_t col) const {
    CudaVector<FloatT, Rows> vec;
    for (std::size_t row = 0; row < Rows; ++row) {
      vec(row) = (*this)(row, col);
    }
    return vec;
  }

  __host__ __device__
  CudaVector<FloatT, Cols> row(const std::size_t row) const {
    CudaVector<FloatT, Cols> vec;
    for (std::size_t col = 0; col < Cols; ++col) {
      vec(col) = (*this)(row, col);
    }
    return vec;
  }

  template<std::size_t row_offset, std::size_t col_offset, std::size_t row_length, std::size_t col_length>
  __host__ __device__
  CudaMatrix<FloatT, row_length, col_length> block() const {
    CudaMatrix<FloatT, row_length, col_length> block_matrix;
//    static_assert(row_offset + row_length <= Rows);
//    static_assert(col_offset + col_length <= Cols);
    for (std::size_t row = row_offset; row < row_offset + row_length; ++row) {
      for (std::size_t col = col_offset; col < col_offset + col_length; ++col) {
        block_matrix(row - row_offset, col - col_offset) = (*this)(row, col);
      }
    }
    return block_matrix;
  }

  __host__ __device__
  CudaVector<FloatT, Rows> operator*(const CudaVector<FloatT, Cols> &vec) {
    CudaVector<FloatT, Rows> result_vec = CudaVector<FloatT, Rows>::Zero();
    for (std::size_t row = 0; row < Rows; ++row) {
      for (std::size_t col = 0; col < Cols; ++col) {
        result_vec(row) += (*this)(row, col) * vec(col);
      }
    }
    return result_vec;
  }

  __host__ __device__
  const FloatT &operator()(const std::size_t index) const {
    return matrix_[index];
  }

  __host__ __device__
  FloatT &operator()(const std::size_t index) {
    return matrix_[index];
  }

  __host__ __device__
  const FloatT &operator()(const std::size_t row, const std::size_t col) const {
    return matrix_[row * Cols + col];
  }

  __host__ __device__
  FloatT &operator()(const std::size_t row, const std::size_t col) {
    return matrix_[row * Cols + col];
  }

  __host__ __device__
  void copyFrom(const FloatT *data, const StorageOrder storage_order = RowMajor) {
    if (storage_order == RowMajor) {
      for (std::size_t i = 0; i < size(); ++i) {
        matrix_[i] = data[i];
      }
    }
    else if (storage_order == ColumnMajor) {
      for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t col = 0; col < Cols; ++col) {
          (*this)(row, col) = data[col * Rows + row];
        }
      }
    }
  }

  __host__ __device__
  void copyTo(FloatT *data, const StorageOrder storage_order = RowMajor) const {
    if (storage_order == RowMajor) {
      for (std::size_t i = 0; i < size(); ++i) {
        data[i] = (*this)(i);
      }
    }
    else if (storage_order == ColumnMajor) {
      for (std::size_t row = 0; row < Rows; ++row) {
        for (std::size_t col = 0; col < Cols; ++col) {
          data[col * Rows + row] = (*this)(row, col);
        }
      }
    }
  }

  __host__ __device__
  CudaMatrix cwiseMin(const CudaMatrix &other) const {
    CudaMatrix mat;
    for (std::size_t i = 0; i < size(); ++i) {
      mat(i) = std::min(matrix_[i], other(i));
    }
    return mat;
  }

  __host__ __device__
  CudaMatrix cwiseMax(const CudaMatrix &other) const {
    CudaMatrix vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = std::max(matrix_[i], other(i));
    }
    return vec;
  }

  __host__ __device__
  CudaMatrix cwiseInverse() const {
    CudaMatrix vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = (FloatT) 1 / matrix_[i];
    }
    return vec;
  }

  __host__ __device__
  CudaMatrix operator+(const CudaMatrix &other) const {
    CudaMatrix vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = matrix_[i] + other(i);
    }
    return vec;
  }

  __host__ __device__
  CudaMatrix operator-(const CudaMatrix &other) const {
    CudaMatrix vec;
    for (std::size_t i = 0; i < Rows; ++i) {
      vec(i) = matrix_[i] - other(i);
    }
    return vec;
  }

  __host__ __device__
  CudaMatrix operator*(const FloatT &scalar) const {
    CudaMatrix mat;
    for (std::size_t i = 0; i < size(); ++i) {
      mat(i) = matrix_[i] * scalar;
    }
    return mat;
  }

  __host__ __device__
  void print() const {
    for (std::size_t row = 0; row < Rows; ++row) {
      for (std::size_t col = 0; col < Cols; ++col) {
        printf("%f ", (*this)(row, col));
      }
      printf("\n");
    }
    printf("\n");
  }

private:
  FloatT matrix_[Rows * Cols];
};

}
