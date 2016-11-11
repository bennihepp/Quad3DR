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

#ifndef _MSC_VER
#define NOEXCEPT noexcept
#else
#define NOEXCEPT
#endif


#define FUNCTION_LINE_STRING (std::string(__FILE__) + " [" + std::string(__FUNCTION__) + ":" + std::to_string(__LINE__) + "]")

#define ANNOTATE_EXC(type, s) type (std::string(FUNCTION_LINE_STRING).append(": ").append(s))

#ifndef AIT_EXCEPTION
    #define AIT_EXCEPTION(s) ait::Exception(std::string(FUNCTION_LINE_STRING).append(": ").append(s).c_str())
#endif

#define MLIB_WARNING(s) ait::warningFunction(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s))

#define MLIB_ERROR(s) ait::errorFunction(std::string(FUNCTION_LINE_STRING) + ": " + std::string(s))


#if defined(DEBUG) || defined(_DEBUG)
#define AIT_ASSERT_STR(b,s) { if(!(b)) ait::assertFunction(b, std::string(FUNCTION_LINE_STRING) + ": " + std::string(s)); }
#define AIT_ASSERT(b) { if(!(b)) ait::assertFunction(b, FUNCTION_LINE_STRING); }
#else
#define AIT_ASSERT_STR(b,s)
#define AIT_ASSERT(b)
#endif


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


class RateCounter
{
public:
    using clock = std::chrono::high_resolution_clock;

    RateCounter(unsigned int report_count = 10)
        : report_count_(report_count) {
        reset();
    }

    void reset() {
        counter_ = 0;
        start_time_ = clock::now();
    }

    void count() {
        ++counter_;
    }

    unsigned int getCount() const {
        return counter_;
    }

    double getRate() const {
        auto cur_time = clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);
        double rate = counter_ / (0.001 * duration_ms.count());
        return rate;
    }

    bool reportRate(double& rate) {
        if (counter_ >= report_count_) {
            auto cur_time = clock::now();
            auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - start_time_);
            rate = counter_ / (0.001 * duration_ms.count());
            start_time_ = cur_time;
            counter_ = 0;
            return true;
        }
        else {
            return false;
        }
    }

private:
    unsigned int counter_;
    unsigned int report_count_;
    std::chrono::time_point<clock> start_time_;
};

class PaceMaker
{
public:
    using clock = std::chrono::high_resolution_clock;
    PaceMaker(double desired_rate)
        : desired_period_(1 / desired_rate), time_ahead_(0) {
        last_time_ = clock::now();
    }

    void sleep() {
        std::chrono::time_point<clock> now = clock::now();
        std::chrono::duration<double> duration = now - last_time_;
        time_ahead_ = time_ahead_ + desired_period_ - duration;
        if (time_ahead_.count() < 0) {
            time_ahead_ = std::chrono::duration<double>(0);
        }
        std::chrono::milliseconds sleep_time_ms(static_cast<int64_t>(1000 * time_ahead_.count()));
        std::this_thread::sleep_for(sleep_time_ms);
        last_time_ = now;
    }

private:
    std::chrono::time_point<clock> last_time_;
    std::chrono::duration<double> desired_period_;
    std::chrono::duration<double> time_ahead_;
};

}
