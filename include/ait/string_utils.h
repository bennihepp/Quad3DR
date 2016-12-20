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

namespace ait {

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

}
