//==================================================
// common.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 15, 2017
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
#define BH_PRINT_VALUE(x) std::cout << #x << "=" << (x) << std::endl

// Marco for function name and line number as a string
#define FUNCTION_LINE_STRING (std::string(__FILE__) + " [" + std::string(__FUNCTION__) + ":" + std::to_string(__LINE__) + "]")

// Exceptions
#define ANNOTATE_EXC(type, s) type (std::string(FUNCTION_LINE_STRING).append(": ").append(s))

#ifndef BH_EXCEPTION
  #define BH_EXCEPTION(s) bh::Exception(std::string(FUNCTION_LINE_STRING).append(": ").append(s).c_str())
#endif

// Warning and error output
#if !defined(MLIB_WARNING) && BH_MLIB_COMPATIBILITY
  #define MLIB_WARNING(s) bh::warningFunction(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s))
#endif

#if !defined(MLIB_ERROR) && BH_MLIB_COMPATIBILITY
  #define MLIB_ERROR(s) bh::errorFunction(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s))
#endif

#define BH_DEBUG_BREAK bh::debugBreakFunction()

// Assertion macros
#if !BH_NO_ASSERT
  #define BH_ASSERT_STR(b, s) { if (!(b)) { bh::assertMessage(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s)); } }
  #define BH_ASSERT(b) { if (!(b)) { bh::assertMessage(FUNCTION_LINE_STRING); } }
  #if defined(DEBUG) || defined(_DEBUG)
    #define BH_ASSERT_DBG_STR(b, s) { if (!(b)) { bh::assertMessage(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s)); } }
    #define BH_ASSERT_DBG(b) { if (!(b)) { bh::assertMessage(FUNCTION_LINE_STRING); } }
  #else
    #define BH_ASSERT_DBG_STR(b, s)
    #define BH_ASSERT_DBG(b)
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


namespace bh {

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
