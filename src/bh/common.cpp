//==================================================
// common.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 8, 2016
//==================================================

#include <csignal>
#include <string>
#include <iostream>

#include <bh/common.h>

namespace bh {

#pragma GCC push_options
#pragma GCC optimize("O0")
void debugBreakFunction() {
  #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    __debugbreak();
  #else
    std::raise(SIGINT);
  #endif
}
#pragma GCC pop_options

void warningFunction(const std::string& description) {
	std::cout << "WARNING: " << description << std::endl;
}

void errorFunction(const std::string &description)
{
  std::cerr << "ERROR: " << description << std::endl;
#if _DEBUG || BH_DEBUG || BH_ASSERT_BREAK
  #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
    __debugbreak();
  #else
    std::raise(SIGINT);
  #endif
#endif
}

void assertFunction(bool predicate, const std::string& description) {
  if(!predicate) {
    assertMessage(description);
  }
}

void assertMessage(const std::string& description) {
  std::cerr << "Assertion failed. " << description << std::endl;
#if _DEBUG || BH_DEBUG || BH_ASSERT_BREAK
  #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
  __debugbreak();
  #else
  std::raise(SIGINT);
  #endif
#else
  throw BH_EXCEPTION(std::string("Assertion failed: ") + description);
#endif
}

}
