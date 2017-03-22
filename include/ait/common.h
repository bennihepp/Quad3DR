//==================================================
// common.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 8, 2016
//==================================================

#pragma once

#include <stdexcept>
#include <chrono>
#include <thread>
#include <algorithm>

#ifndef _MSC_VER
#define NOEXCEPT noexcept
#else
#define NOEXCEPT
#endif

// Easy value printer
#define AIT_PRINT_VALUE(x) std::cout << #x << "=" << x << std::endl

// Marco for function name and line number as a string
#define FUNCTION_LINE_STRING (std::string(__FILE__) + " [" + std::string(__FUNCTION__) + ":" + std::to_string(__LINE__) + "]")

// Exceptions
#define ANNOTATE_EXC(type, s) type (std::string(FUNCTION_LINE_STRING).append(": ").append(s))

#ifndef AIT_EXCEPTION
  #define AIT_EXCEPTION(s) ait::Exception(std::string(FUNCTION_LINE_STRING).append(": ").append(s).c_str())
#endif

// Warning and error output
#if !defined(MLIB_WARNING) && AIT_MLIB_COMPATIBILITY
  #define MLIB_WARNING(s) ait::warningFunction(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s))
#endif

#if !defined(MLIB_ERROR) && AIT_MLIB_COMPATIBILITY
  #define MLIB_ERROR(s) ait::errorFunction(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s))
#endif

#define AIT_DEBUG_BREAK ait::debugBreakFunction()

// Assertion macros
#if !AIT_NO_ASSERT
  #define AIT_ASSERT_STR(b, s) { if (!(b)) { ait::assertMessage(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s)); } }
  #define AIT_ASSERT(b) { if (!(b)) { ait::assertMessage(FUNCTION_LINE_STRING); } }
  #if defined(DEBUG) || defined(_DEBUG)
    #define AIT_ASSERT_DBG_STR(b, s) { if (!(b)) { ait::assertMessage(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s)); } }
    #define AIT_ASSERT_DBG(b) { if (!(b)) { ait::assertMessage(FUNCTION_LINE_STRING); } }
  #else
    #define AIT_ASSERT_DBG_STR(b, s)
    #define AIT_ASSERT_DBG(b)
  #endif
#endif

// Safe delete and free macros
#ifndef SAFE_DELETE
    #define SAFE_DELETE(p)       { if (p != nullptr) { delete (p); (p) = nullptr; } }
#endif

#ifndef SAFE_DELETE_ARRAY
    #define SAFE_DELETE_ARRAY(p) { if (p != nullptr) { delete[] (p);   (p) = nullptr; } }
#endif

#ifndef SAFE_FREE
    #define SAFE_FREE(p) { if (p != nullptr) { std::free(p);   (p) = nullptr; } }
#endif


namespace ait
{

class Exception : public std::exception {
public:
    Exception(const std::string& what)
    : std::exception() {
        what_ = what;
    }

    Exception(const char* what)
    : std::exception() {
        what_ = std::string(what);
    }

    const char* what() const NOEXCEPT {
        return what_.c_str();
    }

private:
    std::string what_;
};

class Error : public Exception {
public:
    Error(const std::string& what)
    : Exception(what) {
    }

    Error(const char* what)
    : Exception(what) {
    }
};

void debugBreakFunction();
void warningFunction(const std::string& description);
void errorFunction(const std::string& description);
void assertFunction(bool predicate, const std::string& description);
void assertMessage(const std::string& description);

}

#include <csignal>
#include <iostream>

namespace ait {

#if __GNUC__ && !__CUDACC__
#pragma GCC push_options
#pragma GCC optimize("O0")
#endif
inline void debugBreakFunction() {
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
  __debugbreak();
#else
  std::raise(SIGINT);
#endif
}
#if __GNUC__ && !__CUDACC__
#pragma GCC pop_options
#endif

inline void warningFunction(const std::string& description) {
  std::cout << "WARNING: " << description << std::endl;
}

inline void errorFunction(const std::string &description)
{
  std::cerr << "ERROR: " << description << std::endl;
#if _DEBUG || AIT_DEBUG || AIT_ASSERT_BREAK
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
  __debugbreak();
#else
  std::raise(SIGINT);
#endif
#endif
}

inline void assertFunction(bool predicate, const std::string& description) {
  if(!predicate) {
    assertMessage(description);
  }
}

inline void assertMessage(const std::string& description) {
  std::cerr << "Assertion failed. " << description << std::endl;
#if _DEBUG || AIT_DEBUG || AIT_ASSERT_BREAK
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
  __debugbreak();
#else
  std::raise(SIGINT);
#endif
#else
  throw AIT_EXCEPTION(std::string("Assertion failed: ") + description);
#endif
}

}