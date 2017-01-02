/*
 * eigen_serialization.h
 *
 *  Created on: Jan 2, 2017
 *      Author: bhepp
 */
#pragma once

#include <boost/serialization/array.hpp>
#include "eigen.h"

namespace boost {
//namespace serialization {

// TODO: These don't work
//  template<typename Archive, typename Derived>
//  inline void serialize(
//      Archive& ar,
//      Eigen::EigenBase<Derived>& matrix,
//      const unsigned int version
//  ) {
//    ar & boost::serialization::make_array(matrix.data(), matrix.size());
//  }
//
//  template<typename Archive, typename Derived>
//  inline void serialize(
//      Archive& ar,
//      const Eigen::EigenBase<Derived>& matrix,
//      const unsigned int version
//  ) {
//    ar & boost::serialization::make_array(matrix.data(), matrix.size());
//  }

//  template<typename Archive, typename Derived>
//  inline void serialize(
//      Archive& ar,
//      Eigen::QuaternionBase<Derived>& quat,
//      const unsigned int version
//  ) {
//    ar & boost::serialization::make_array(quat.coeffs().data(), quat.coeffs().size());
//  }
//
//  template<typename Archive, typename Derived>
//  inline void serialize(
//      Archive& ar,
//      const Eigen::QuaternionBase<Derived>& quat,
//      const unsigned int version
//  ) {
//    ar & boost::serialization::make_array(quat.coeffs().data(), quat.coeffs().size());
//  }

  template<typename Archive>
  inline void serialize(
      Archive& ar,
      Eigen::Quaternion<float>& quat,
      const unsigned int version
  ) {
    ar & boost::serialization::make_array(quat.coeffs().data(), quat.coeffs().size());
  }

  template<typename Archive>
  inline void serialize(
      Archive& ar,
      const Eigen::Quaternion<float>& quat,
      const unsigned int version
  ) {
    ar & boost::serialization::make_array(quat.coeffs().data(), quat.coeffs().size());
  }

  template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
  inline void serialize(
      Archive & ar,
      Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix,
      const unsigned int file_version) {
    ar & boost::serialization::make_array(matrix.data(), matrix.size());
  }

  template<class Archive, typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
  inline void serialize(
      Archive & ar,
      const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix,
      const unsigned int file_version) {
    ar & boost::serialization::make_array(matrix.data(), matrix.size());
  }

//}
}
