//==================================================
// eigen_options.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Feb 21, 2017
//==================================================

#pragma once

#include "eigen.h"
#include <boost/program_options.hpp>
#include <boost/throw_exception.hpp>
#include <boost/algorithm/string.hpp>
#include <type_traits>

namespace boost {

template <typename Scalar, int Rows, int Cols>
void validate(boost::any& v,
              const std::vector<std::string>& values,
              Eigen::Matrix<Scalar, Rows, Cols>* target_type, int) {
  using namespace boost::program_options;

  // Make sure no previous assignment to 'v' was made.
  validators::check_first_occurrence(v);

  const std::string& s = validators::get_single_string(values);

  std::vector<std::string> split_vec;
  boost::split(split_vec, s, boost::is_any_of(" ,"), boost::token_compress_on);

  if (split_vec.size() != Rows * Cols) {
    boost::throw_exception(validation_error(validation_error::invalid_option_value));
  }

  typename Eigen::Matrix<Scalar, Rows, Cols> matrix;
  for (size_t i = 0; i < static_cast<size_t>(matrix.size()); ++i) {
    matrix(i) = boost::lexical_cast<Scalar>(split_vec[i]);
  }
  v = boost::any(matrix);
}

}
