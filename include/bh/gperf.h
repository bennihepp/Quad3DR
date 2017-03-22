//==================================================
// gperf.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 21.03.17
//

#pragma once

#if BH_WITH_PROFILING
#include <gperftools/profiler.h>
#endif

namespace bh {

inline void ProfilerStart(const std::string &filename) {
#if BH_WITH_PROFILING
  ::ProfilerStart(filename.c_str());
#endif
}

inline void ProfilerStop() {
#if BH_WITH_PROFILING
  ::ProfilerStop();
#endif
}

}
