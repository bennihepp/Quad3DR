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

#include <ait/common.h>

namespace ait
{

void warningFunction(const std::string& description)
{
	std::cout << "WARNING: " << description << std::endl;
}

void errorFunction(const std::string &description)
{
    std::cout << description << std::endl;
#ifdef _DEBUG
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        __debugbreak();
    #else
        std::raise(SIGINT);
    #endif
#endif
}

void assertFunction(bool predicate, const std::string& description)
{
    if(!predicate) {
        std::cerr << "Assertion failed. " << description << std::endl;
#ifdef _DEBUG
    #if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
        __debugbreak();
    #else
        std::raise(SIGINT);
    #endif
#endif
    }
}

}
