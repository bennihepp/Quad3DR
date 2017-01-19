//==================================================
// qt_utils.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 18, 2017
//==================================================

#pragma once

#include <QVector3D>
#include <QMatrix4x4>

std::ostream& operator<<(std::ostream& out, const QVector3D& vec);

std::ostream& operator<<(std::ostream& out, const QMatrix4x4& mat);

inline std::ostream& operator<<(std::ostream& out, const QVector3D& vec) {
  out << "(" << vec.x() << ", " << vec.y() << ", " << vec.z() << ")";
  return out;
}

inline std::ostream& operator<<(std::ostream& out, const QMatrix4x4& mat) {
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      if (j > 0)
        std::cout << ", ";
      std::cout << mat(i, j);
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  return out;
}
