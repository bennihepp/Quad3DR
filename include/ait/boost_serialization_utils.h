//==================================================
// boost_serialization_utils.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Feb 28, 2017
//==================================================

#pragma once

#include <tuple>

// std::tuple serialization
// by Christopher Allen Ogden (https://github.com/Sydius/serialize-tuple)
//    Copyright 2011 Christopher Allen Ogden. All rights reserved.
//
//    Redistribution and use in source and binary forms, with or without modification, are
//    permitted provided that the following conditions are met:
//
//       1. Redistributions of source code must retain the above copyright notice, this list of
//          conditions and the following disclaimer.
//
//       2. Redistributions in binary form must reproduce the above copyright notice, this list
//          of conditions and the following disclaimer in the documentation and/or other materials
//          provided with the distribution.
//
//    THIS SOFTWARE IS PROVIDED BY CHRISTOPHER ALLEN OGDEN ``AS IS'' AND ANY EXPRESS OR IMPLIED
//    WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
//    FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL CHRISTOPHER ALLEN OGDEN OR
//    CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//    CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
//    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
//    NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
//    ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//    The views and conclusions contained in the software and documentation are those of the
//    authors and should not be interpreted as representing official policies, either expressed
//    or implied, of Christopher Allen Ogden.

namespace boost {
namespace serialization {

template<uint N>
struct serialize_tuple {
  template<class Archive, typename... Args>
  static void serialize(Archive& ar, std::tuple<Args...>& t, const unsigned int version) {
    ar & std::get<N-1>(t);
    serialize_tuple<N-1>::serialize(ar, t, version);
  }
};

template<>
struct serialize_tuple<0> {
  template<class Archive, typename... Args>
  static void serialize(Archive& ar, std::tuple<Args...>& t, const unsigned int version) {
  }
};

template<class Archive, typename... Args>
void serialize(Archive& ar, std::tuple<Args...>& t, const unsigned int version)
{
  serialize_tuple<sizeof...(Args)>::serialize(ar, t, version);
}

}
}
