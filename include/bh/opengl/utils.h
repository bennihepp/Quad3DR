//==================================================
// utils.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 19.04.17
//==================================================
#pragma once

#include <bh/color.h>
#include <bh/qt/utils.h>
#include <QColor>
#include <QImage>

namespace bh {
namespace opengl {

template <typename FloatT>
size_t colorToIndex(const bh::Color3<FloatT> &color);

template <typename FloatT>
size_t colorToIndex(const bh::Color4<FloatT> &color);

bh::Color4<float> indexToColor(const size_t index);

bh::Color3<float> indexToColor3(const size_t index);

//template <typename FloatT>
//bh::Color4<float> floatToColor(const FloatT depth) const;

//template <typename FloatT>
//FloatT colorToFloat(const bh::Color4<float>& color) const;

//inline FloatType colorToFloat(const QImage& float_image, const size_t x, size_t y) const;

//inline FloatType colorToFloat(const QImage& float_image, const Vector2& image_point) const;

QImage convertIndicesImageToRGB(
        const QImage& encoded_image, const size_t min_index, const size_t max_index);

//QImage convertFloatImageToRGB(
//        const QImage& encoded_image, const FloatType min_value, const FloatType max_value) const;

std::unordered_set<size_t> getIndicesSetFromImage(const QImage& indices_image);

template <typename FloatT>
size_t colorToIndex(const bh::Color3<FloatT> &color) {
  return (static_cast<size_t>(color.r() * FloatT(255)) << 0) +
         (static_cast<size_t>(color.g() * FloatT(255)) << 8) +
         (static_cast<size_t>(color.b() * FloatT(255)) << 16);
}

template <typename FloatT>
size_t colorToIndex(const bh::Color4<FloatT> &color) {
  return (static_cast<size_t>(color.r() * FloatT(255)) << 0) +
         (static_cast<size_t>(color.g() * FloatT(255)) << 8) +
         (static_cast<size_t>(color.b() * FloatT(255)) << 16);
}

inline bh::Color4<float> indexToColor(const size_t index) {
  const float r = ((index & 0x000000FF) >> 0) / float(255);
  const float g = ((index & 0x0000FF00) >> 8) / float(255);
  const float b = ((index & 0x00FF0000) >> 16) / float(255);
  const float a = 1;
  return bh::Color4<float>(r, g, b, a);
}

inline bh::Color3<float> indexToColor3(const size_t index) {
  const float r = ((index & 0x000000FF) >> 0) / float(255);
  const float g = ((index & 0x0000FF00) >> 8) / float(255);
  const float b = ((index & 0x00FF0000) >> 16) / float(255);
  return bh::Color3<float>(r, g, b);
}

inline QImage convertIndicesImageToRGB(
        const QImage& encoded_image, const size_t min_index, const size_t max_index) {
  QImage rgb_image(encoded_image.width(), encoded_image.height(), QImage::Format_ARGB32);
  bh::ColorMapHot<float> cmap;
  for (int y = 0; y < encoded_image.height(); ++y) {
    for (int x = 0; x < encoded_image.width(); ++x) {
      const QColor color_qt(encoded_image.pixel(x, y));
      const bh::Color4<float> color = bh::qt::colorFromQt<float>(color_qt);
      const size_t index = colorToIndex(color);
      const size_t clamped_index = bh::clamp(index, min_index, max_index);
      const bh::Color3<float> color_cmap = cmap.map(bh::normalize<float>(clamped_index, min_index, max_index));
      const QColor pixel(
              bh::clamp<int>(255 * color_cmap.r(), 0, 255),
              bh::clamp<int>(255 * color_cmap.g(), 0, 255),
              bh::clamp<int>(255 * color_cmap.b(), 0, 255));
      rgb_image.setPixel(x, y, pixel.rgba());
    }
  }
  return rgb_image;
}

inline std::unordered_set<size_t> getIndicesSetFromImage(const QImage& indices_image) {
  const bh::Color4<float> invalid_color(1, 1, 1, 1);
  const size_t invalid_index = colorToIndex(invalid_color);
  std::unordered_set<size_t> indices_set;
  for (int y = 0; y < indices_image.height(); ++y) {
    for (int x = 0; x < indices_image.width(); ++x) {
      const QColor color_qt(indices_image.pixel(x, y));
      const bh::Color4<float> color = bh::qt::colorFromQt<float>(color_qt);
      const size_t index = colorToIndex(color);
      if (index != invalid_index) {
        indices_set.emplace(index);
      }
    }
  }
  return indices_set;
}

}
}
