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

namespace bh {

namespace boostfs = boost::filesystem;

////////////////////////
// File system utilities
////////////////////////

template <typename... T>
std::string joinPaths(T const&... paths) {
  boostfs::path result;
  bool _[]{false, (result /= boostfs::path(paths), false)...};
  static_cast<void>(_);
  return result.string();
}

//
// Thanks to http://stackoverflow.com/questions/10167382/boostfilesystem-get-relative-path.
//
inline boostfs::path pathRelativeTo(const boostfs::path& from, const boostfs::path& to, const bool make_absolute = true) {
  if (make_absolute) {
    const boostfs::path abs_from = boostfs::absolute(from);
    const boostfs::path abs_to = boostfs::absolute(to);
    return pathRelativeTo(abs_from, abs_to, false);
  }

  // Start at the root path and while they are the same then do nothing then when they first
  // diverge take the remainder of the two path and replace the entire from path with ".."
  // segments.
  boostfs::path::const_iterator from_iter = from.begin();
  boostfs::path::const_iterator to_iter = to.begin();

  // Loop through both while same
  while (from_iter != from.end() && to_iter != to.end() && (*to_iter) == (*from_iter)) {
    ++to_iter;
    ++from_iter;
  }

  boostfs::path final_path;
  while (from_iter != from.end()) {
    final_path /= "..";
    ++from_iter;
  }

  while (to_iter != to.end()) {
    final_path /= *to_iter;
    ++to_iter;
  }

  return final_path;
}

}
