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

// Marco for function name and line number as a string
#define FUNCTION_LINE_STRING (std::string(__FILE__) + " [" + std::string(__FUNCTION__) + ":" + std::to_string(__LINE__) + "]")

// Exceptions
#define ANNOTATE_EXC(type, s) type (std::string(FUNCTION_LINE_STRING).append(": ").append(s))

#ifndef AIT_EXCEPTION
    #define AIT_EXCEPTION(s) ait::Exception(std::string(FUNCTION_LINE_STRING).append(": ").append(s).c_str())
#endif

// Warning and error output
#if !defined(MLIB_WARNING) && !AIT_MLIB_COMPATIBILITY
#define MLIB_WARNING(s) ait::warningFunction(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s))
#endif

#if !defined(MLIB_ERROR) && !AIT_MLIB_COMPATIBILITY
#define MLIB_ERROR(s) ait::errorFunction(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s))
#endif

// Assertion macros
#define AIT_ASSERT_STR(b, s) { if (!(b)) { ait::assertMessage(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s)); } }
#define AIT_ASSERT(b) { if (!(b)) { ait::assertMessage(FUNCTION_LINE_STRING); } }
#if defined(DEBUG) || defined(_DEBUG)
#define AIT_ASSERT_DBG_STR(b, s) { if (!(b)) { ait::assertMessage(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s)); } }
#define AIT_ASSERT_DBG(b) { if (!(b)) { ait::assertMessage(FUNCTION_LINE_STRING); } }
#else
#define AIT_ASSERT_DBG_STR(b, s)
#define AIT_ASSERT_DBG(b)
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

void warningFunction(const std::string& description);
void errorFunction(const std::string& description);
void assertFunction(bool predicate, const std::string& description);
void assertMessage(const std::string& description);

}
