/*
 * color.h
 *
 *  Created on: Dec 31, 2016
 *      Author: bhepp
 */
#pragma once

#include "common.h"
#include "utilities.h"
#include "eigen.h"
#include "eigen_utils.h"

namespace ait {

template <typename FloatType>
void convertHsvToRgb(const FloatType h, const FloatType s, const FloatType v, FloatType* r, FloatType* g,FloatType* b) {
  *r = std::abs(h * 6 - 3) - 1;
  *g = 2 - std::abs(h * 6 - 2);
  *b = 2 - std::abs(h * 6 - 4);
  *r = ait::clamp(*r);
  *g = ait::clamp(*g);
  *b = ait::clamp(*b);
  *r = ((*r - 1) * s + 1) * v;
  *g = ((*g - 1) * s + 1) * v;
  *b = ((*b - 1) * s + 1) * v;;
}

template <typename FloatType>
class Color3;

template <typename FloatType>
class Color4;

template <typename FloatType>
class Color3HSV;

template <typename FloatType>
class Color4HSV;

template <typename FloatType>
class Color4 : public Eigen::Matrix<FloatType, 4, 1> {
public:
  using Base = Eigen::Matrix<FloatType, 4, 1>;

  Color4()
  : Base(0, 0, 0, 1) {}

  Color4(const FloatType r, const FloatType g, const FloatType b, const FloatType a)
  : Base(r, g, b, a) {}

  static Color4 createFromColor3(const Color3<FloatType>& color3, const FloatType a) {
    return Color4(color3.r(), color3.g(), color3.b(), a);
  }

  static Color4 createFromColorHSV(const Color4HSV<FloatType>& hsv) {
//    FloatType r = std::abs(hsv.h() * 6 - 3) - 1;
//    FloatType g = 2 - std::abs(hsv.h() * 6 - 2);
//    FloatType b = 2 - std::abs(H * 6 - 4);
//    FloatType a = hsv.a();
//    Color4 color(r, g, b, a);
//    color = ait::clamp(color);
//    color = ((color - 1) * hsv.s() + 1) * hsv.v();
    Color4 rgb;
    convertHsvToRgb(hsv.h(), hsv.s(), hsv.v(), &rgb.r(), &rgb.g(), &rgb.b());
    rgb.a() = hsv.a();
    return rgb;
  }

  const FloatType& r() const {
    return (*this)(0);
  }

  FloatType& r() {
    return (*this)(0);
  }

  const FloatType& g() const {
    return (*this)(1);
  }

  FloatType& g() {
    return (*this)(1);
  }

  const FloatType& b() const {
    return (*this)(2);
  }

  FloatType& b() {
    return (*this)(2);
  }

  const FloatType& a() const {
    return (*this)(3);
  }

  FloatType& a() {
    return (*this)(3);
  }

  template <typename OtherFloatType>
  Color3<OtherFloatType> cast() const {
    return Color3<OtherFloatType>(
        static_cast<OtherFloatType>(r()),
        static_cast<OtherFloatType>(g()),
        static_cast<OtherFloatType>(b()));
  }

private:
};

template <typename FloatType>
class Color3 : public Eigen::Matrix<FloatType, 3, 1> {
public:
  using Base = Eigen::Matrix<FloatType, 3, 1>;

  Color3()
  : Base(0, 0, 0) {}

  Color3(const FloatType r, const FloatType g, const FloatType b)
  : Base(r, g, b) {}

  static Color3 createFromColor4RGB(const Color4<FloatType>& color4) {
    return Color3(color4.r(), color4.g(), color4.b());
  }

  static Color3 createFromColorHSV(const Color3HSV<FloatType>& hsv) {
    Color3 rgb;
    convertHsvToRgb(hsv.h(), hsv.s(), hsv.v(), &rgb.r(), &rgb.g(), &rgb.b());
    return rgb;
  }

  const FloatType& r() const {
    return (*this)(0);
  }

  FloatType& r() {
    return (*this)(0);
  }

  const FloatType& g() const {
    return (*this)(1);
  }

  FloatType& g() {
    return (*this)(1);
  }

  const FloatType& b() const {
    return (*this)(2);
  }

  FloatType& b() {
    return (*this)(2);
  }

  template <typename OtherFloatType>
  Color3<OtherFloatType> cast() const {
    return Color3<OtherFloatType>(
        static_cast<OtherFloatType>(r()),
        static_cast<OtherFloatType>(g()),
        static_cast<OtherFloatType>(b()));
  }

private:
};

template <typename FloatType>
void convertRgbToHsv(const FloatType r, const FloatType g, const FloatType b, FloatType* h, FloatType* s,FloatType* v) {
  FloatType rgb_min;
  FloatType rgb_max;
  rgb_min = r < g ? (r < b ? r : b) : (g < b ? g : b);
  rgb_max = r > g ? (r > b ? r : b) : (g > b ? g : b);
  *v = rgb_max;
  if (*h == 0) {
    *h = 0;
    *s = 0;
    return;
  }

  *s = (rgb_max - rgb_min) / *v;
  if (*s == 0) {
    *h = 0;
    return;
  }

  if (rgb_max == r) {
    *h = 0 + (FloatType)(43/255.) * (g - b) / (rgb_max - rgb_min);
  }
  else if (rgb_max == g) {
    *h = (FloatType)(85/255.) + (FloatType)(43/255.) * (b - r) / (rgb_max - rgb_min);
  }
  else {
    *h = (FloatType)(171/255.) + (FloatType)(43/255.) * (r - g) / (rgb_max - rgb_min);
  }
}

template <typename FloatType>
class Color3HSV;

template <typename FloatType>
class Color4HSV : public Eigen::Matrix<FloatType, 4, 1> {
public:
  using Base = Eigen::Matrix<FloatType, 4, 1>;

  Color4HSV()
  : Base(0, 0, 0) {}

  Color4HSV(const FloatType h, const FloatType s, const FloatType v, const FloatType a)
  : Base(h, s, v, a) {}

  static Color4HSV createFromColorRGB(const Color4<FloatType>& rgb) {
    Color4HSV hsv;
    convertRgbToHsv(rgb.r(), rgb.g(), rgb.b(), &hsv.h(), &hsv.s(), &hsv.v());
    hsv.a() = rgb.a();
    return hsv;
  }

  static Color4HSV createFromColor3HSV(const Color3HSV<FloatType>& color3, const FloatType a) {
    return Color4HSV(color3.h(), color3.s(), color3.v(), a);
  }

  const FloatType& h() const {
    return (*this)(0);
  }

  FloatType& h() {
    return (*this)(0);
  }

  const FloatType& s() const {
    return (*this)(1);
  }

  FloatType& s() {
    return (*this)(1);
  }

  const FloatType& v() const {
    return (*this)(2);
  }

  FloatType& v() {
    return (*this)(2);
  }

  const FloatType& a() const {
    return (*this)(3);
  }

  FloatType& a() {
    return (*this)(3);
  }

  template <typename OtherFloatType>
  Color3HSV<OtherFloatType> cast() const {
    return Color3<OtherFloatType>(
        static_cast<OtherFloatType>(h()),
        static_cast<OtherFloatType>(s()),
        static_cast<OtherFloatType>(v()));
  }

private:
};

template <typename FloatType>
class Color3HSV : public Eigen::Matrix<FloatType, 3, 1> {
public:
  using Base = Eigen::Matrix<FloatType, 3, 1>;

  Color3HSV()
  : Base(0, 0, 0) {}

  Color3HSV(const FloatType h, const FloatType s, const FloatType v)
  : Base(h, s, v) {}

  static Color3HSV createFromColorRGB(const Color3<FloatType>& color) {
    Color3HSV hsv;
    convertRgbToHsv(color.r(), color.g(), color.b(), &hsv.h(), &hsv.s(), &hsv.v());
    return hsv;
  }

  static Color3HSV createFromColor4HSV(const Color4HSV<FloatType>& color4) {
    return Color3HSV(color4.h(), color4.s(), color4.v());
  }

  const FloatType& h() const {
    return (*this)(0);
  }

  FloatType& h() {
    return (*this)(0);
  }

  const FloatType& s() const {
    return (*this)(1);
  }

  FloatType& s() {
    return (*this)(1);
  }

  const FloatType& v() const {
    return (*this)(2);
  }

  FloatType& v() {
    return (*this)(2);
  }

  template <typename OtherFloatType>
  Color3HSV<OtherFloatType> cast() const {
    return Color3<OtherFloatType>(
        static_cast<OtherFloatType>(h()),
        static_cast<OtherFloatType>(s()),
        static_cast<OtherFloatType>(v()));
  }

private:
};

template <typename FloatType>
class ColorMapJet;

//template <typename FloatType>
//class ColorMapHot;

template <typename FloatType>
class ColorMapHSV;

template <typename FloatType>
class ColorMap {
public:
  virtual ~ColorMap() {}

  virtual Color3<FloatType> map(FloatType value) = 0;

  virtual Color4<FloatType> map(FloatType value, FloatType a) {
    return Color4<FloatType>::createFromColor3(this->map(value), a);
  }

  static ColorMapJet<FloatType> jet() {
    return ColorMapJet<FloatType>();
  }

//  static ColorMapHot<FloatType> hot() {
//    return ColorMapHot<FloatType>();
//  }

  static ColorMapHSV<FloatType> hsv() {
    return ColorMapHSV<FloatType>();
  }

};

template <typename FloatType>
class ColorMapJet : public ColorMap<FloatType> {
public:
  using Base = ColorMap<FloatType>;

  ~ColorMapJet() override {}

  Color3<FloatType> map(FloatType value) override {
    FloatType r = ait::clamp(std::min(4 * value - (FloatType)1.5, -4 * value + (FloatType)4.5));
    FloatType g = ait::clamp(std::min(4 * value - (FloatType)0.5, -4 * value + (FloatType)3.5));
    FloatType b = ait::clamp(std::min(4 * value + (FloatType)0.5, -4 * value + (FloatType)2.5));
    return Color3<FloatType>(r, g, b);
  }

  Color4<FloatType> map(FloatType value, FloatType a) override {
    return Base::map(value, a);
  }

};

template <typename FloatType>
class ColorMapHot : public ColorMap<FloatType> {
public:
  using Base = ColorMap<FloatType>;

  ~ColorMapHot() override {}

  Color3<FloatType> map(FloatType value) override {
    Color3<FloatType> rgb;
    rgb.r() = ait::clamp(2 * value);
    rgb.g() =  ait::clamp(2 * std::max(value - (FloatType)0.5, (FloatType)0));
    rgb.b() = 0;
    return rgb;
  }

  Color4<FloatType> map(FloatType value, FloatType a) override {
    return Base::map(value, a);
  }

};

template <typename FloatType>
class ColorMapHSV : public ColorMap<FloatType> {
public:
  using Base = ColorMap<FloatType>;

  ~ColorMapHSV() override {}

  Color3<FloatType> map(FloatType value) override {
    return Color3<FloatType>::createFromColorHSV(Color3HSV<FloatType>(value, 1, 1));
  }

  Color4<FloatType> map(FloatType value, FloatType a) override {
    return Base::map(value, a);
  }

};

}
