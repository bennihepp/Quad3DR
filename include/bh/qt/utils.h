//==================================================
// qt_utils.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 18, 2017
//==================================================

#pragma once

#include <bh/color.h>
#include <QVector3D>
#include <QMatrix4x4>
#include <QColor>
#include <QImage>

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

namespace bh {
namespace qt {

template<typename FloatT>
bh::Color3<FloatT> color3FromQt(const QColor &color_qt);

template<typename FloatT>
bh::Color4<FloatT> colorFromQt(const QColor &color_qt);

template<typename FloatT>
QColor colorToQt(const bh::Color3<FloatT> &color);

template<typename FloatT>
QColor colorToQt(const bh::Color4<FloatT> &color);

template<typename FloatT>
bh::Color3<FloatT> color3FromQt(const QColor &color_qt) {
  return bh::Color3<FloatT>(color_qt.redF(), color_qt.greenF(), color_qt.blueF());
}

template<typename FloatT>
bh::Color4<FloatT> colorFromQt(const QColor &color_qt) {
  return bh::Color4<FloatT>(color_qt.redF(), color_qt.greenF(), color_qt.blueF(), color_qt.alphaF());
}

template<typename FloatT>
QColor colorToQt(const bh::Color3<FloatT> &color) {
  return QColor::fromRgbF(color.r(), color.g(), color.b(), 1);
}

template<typename FloatT>
QColor colorToQt(const bh::Color4<FloatT> &color) {
  return QColor::fromRgbF(color.r(), color.g(), color.b(), color.a());
}

inline void setImageAlphaPremultiplied(QImage* image, const qreal alpha) {
  for (int y = 0; y < image->height(); ++y) {
    for (int x = 0; x < image->width(); ++x) {
      QRgb pixel_rgb = qUnpremultiply(image->pixel(x, y));
      QColor pixel_color(pixel_rgb);
      pixel_color.setAlphaF(alpha);
      QRgb new_pixel_color = qPremultiply(pixel_color.rgba());
      image->setPixel(x, y, new_pixel_color);
    }
  }
}

}
}
