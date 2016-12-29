/*
 * mLib.h
 *
 *  Created on: Dec 26, 2016
 *      Author: bhepp
 */

#pragma once

#if _WIN32
  #define NOMINMAX
  #include <WinSock2.h>
  #include <stdio.h>
  #include <windows.h>
#endif

//
// mLib config options
//

#define MLIB_ERROR_CHECK
#define MLIB_BOUNDS_CHECK
#define MLIB_SOCKETS

//
// mLib includes
//

#include "mLibCore.h"
#include "mLibFreeImage.h"
#include "mLibDepthCamera.h"
#include "mLibFLANN.h"
