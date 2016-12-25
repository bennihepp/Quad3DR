//==================================================
// serialization.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 20, 2016
//==================================================
#pragma once

#include <iostream>
#include <ait/common.h>
#include <ait/eigen.h>
#include <type_traits>

namespace ait {

class Serializable {
public:
  virtual ~Serializable() {};

  virtual void write(std::ostream& out) const = 0;

  virtual void read(std::istream& in) = 0;
};

// -------------------------
// File input output utilities
// -------------------------

template <typename T, typename=void>
void writeToStream(std::ostream& out, const T& value);

template <typename T, typename=void>
void readFromStream(std::istream& in, T* value);

template <typename T, typename=void>
T readFromStream(std::istream& in);

template <typename T>
struct is_eigen_type : public std::is_base_of<Eigen::EigenBase<T>, T> {};

template <typename T, typename std::enable_if<ait::is_eigen_type<T>::value>::type>
void writeToStream(std::ostream& out, const T& mat);

template <typename T, typename std::enable_if<ait::is_eigen_type<T>::value>::type>
void readFromStream(std::istream& in, T& mat);

}

// -------------------------
// File input output utilities
// -------------------------

template <typename T, typename=void>
void ait::writeToStream(std::ostream& out, const T& value) {
  out.write(reinterpret_cast<const char*>(&value), sizeof(value));
  if (!static_cast<bool>(out)) {
    throw AIT_EXCEPTION("Unable to write to stream");
  }
}

template <typename T, typename=void>
void ait::readFromStream(std::istream& in, T* value) {
  in.read(reinterpret_cast<char*>(value), sizeof(T));
  if (!static_cast<bool>(in)) {
    throw AIT_EXCEPTION("Unable to read from stream");
  }
}

template <typename T, typename=void>
T ait::readFromStream(std::istream& in) {
  T value;
  in.read(reinterpret_cast<char*>(&value), sizeof(T));
  if (!static_cast<bool>(in)) {
    throw AIT_EXCEPTION("Unable to read from stream");
  }
  return std::move(value);
}

template <typename T, typename std::enable_if<ait::is_eigen_type<T>::value>::type>
void ait::writeToStream(std::ostream& out, const T& mat) {
  for (size_t row = 0; row < mat.rows(); ++row) {
    for (size_t col = 0; col < mat.cols(); ++col) {
      ait::writeToStream<typename T::Scalar>(out, mat(row, col));
    }
  }
}

template <typename T, typename std::enable_if<ait::is_eigen_type<T>::value>::type>
void ait::readFromStream(std::istream& in, T& mat) {
  for (size_t row = 0; row < mat.rows(); ++row) {
    for (size_t col = 0; col < mat.cols(); ++col) {
      mat(row, col) = ait::readFromStream<typename T::Scalar>(in);
    }
  }
}
