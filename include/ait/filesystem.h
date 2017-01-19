//==================================================
// boost_utilities.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 9, 2016
//==================================================
#pragma once

#include "boost.h"
#include <boost/filesystem.hpp>

namespace ait {

////////////////////////
// File system utilities
////////////////////////

template <typename... T>
std::string joinPaths(T const&... paths) {
  boost::filesystem::path result;
  bool _[]{false, (result /= boost::filesystem::path(paths), false)...};
  static_cast<void>(_);
  return result.string();
}

}
