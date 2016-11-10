//==================================================
// mLibInclude.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 14, 2016
//==================================================

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
#include "mLibZLib.h"
//#include "mLibLodePNG.h"
//#include "mLibD3D11.h"

#include "mLibEigen.h"

//move this to mlib (it's currently local)
//#include "mLibCuda.h"

using namespace ml;

