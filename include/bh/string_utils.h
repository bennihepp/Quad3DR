//==================================================
// string_utils.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 7, 2016
//==================================================
#pragma once

#include <string>
#include <cctype>
#include <boost/algorithm/string/find_iterator.hpp>
#include <boost/algorithm/string/finder.hpp>

namespace bh {

template <typename T>
std::vector<T> splitString(const std::string& str, const std::string& delimiter) {
  std::vector<T> result;
  using string_split_iterator = boost::split_iterator<std::string::const_iterator>;
  for (string_split_iterator it = boost::make_split_iterator(str, boost::first_finder(delimiter));
       it != string_split_iterator();
       ++it) {
    const T value = boost::lexical_cast<T>(boost::copy_range<std::string>(*it));
    result.push_back(value);
  }
  return result;
}

template <typename T>
std::vector<T> splitString(const std::string& str, const std::string& delimiter,
                           const std::function<T(const std::string&)>& parser) {
  std::vector<T> result;
  using string_split_iterator = boost::split_iterator<std::string::const_iterator>;
  for (string_split_iterator it = boost::make_split_iterator(str, boost::first_finder(delimiter));
       it != string_split_iterator();
       ++it) {
    const T value = parser(boost::copy_range<std::string>(*it));
    result.push_back(value);
  }
  return result;
}

inline void trimLeft(std::string& str) {
  str.erase(str.begin(), std::find_if(str.begin(), str.end(), [](int ch) { return !isspace(ch); }));
}

inline void trimRight(std::string& str) {
  str.erase(std::find_if(str.rbegin(), str.rend(), [](int ch) { return !isspace(ch); }).base(), str.end());
}

inline void trim(std::string& str) {
  trimLeft(str);
  trimRight(str);
}

/// Remove substring from start of string
inline std::string removeSubstringFromStart(const std::string& str, const std::string& sub_str) {
  const size_t found_pos = str.find(sub_str);
  if (found_pos == std::string::npos) {
    throw Error(std::string("String '") + str + "' does not contain substring '" + sub_str + "'");
  }
  const std::string result = str.substr(found_pos + sub_str.length());
  return result;
}

}
