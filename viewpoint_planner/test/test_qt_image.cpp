//==================================================
// test_qt_image.cpp.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 27.04.17
//==================================================

#include <QCoreApplication>
#include <QImage>
#include <QPainter>
#include <bh/common.h>
#include "gtest/gtest.h"

#pragma GCC optimize("O0")

namespace {
using FloatType = double;
using size_t = std::size_t;

class QtImageTest : public :: testing::Test {
protected:
  QtImageTest() {}

  virtual ~QtImageTest() override {}
};
}

void setImageAlphaPremultiplied(QImage* image, const qreal alpha) {
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

TEST_F(QtImageTest, AlphaBlendImages) {
  const FloatType overlay_alpha = 0.5;
  QImage image1;
  EXPECT_TRUE(image1.load("input1.png"));
//  image1 = image1.convertToFormat(QImage::Format_ARGB32_Premultiplied);
//  EXPECT_TRUE(image1.hasAlphaChannel());
  QImage image2;
  EXPECT_TRUE(image2.load("input2.png"));
//  image2 = image2.convertToFormat(QImage::Format_ARGB32_Premultiplied);
//  EXPECT_TRUE(image2.hasAlphaChannel());
  BH_PRINT_VALUE(image1.width());
  BH_PRINT_VALUE(image1.height());
  BH_PRINT_VALUE(image2.width());
  BH_PRINT_VALUE(image2.height());
//  BH_ASSERT(image1.size() == image2.size());
//  setImageAlphaPremultiplied(&image1, overlay_alpha);
//  setImageAlphaPremultiplied(&image2, overlay_alpha);
//  image1.save("input1_alpha.png");
//  image2.save("input2_alpha.png");
//  QImage overlay_image(image1.width(), image1.height(), QImage::Format_ARGB32_Premultiplied);
  QImage overlay_image = image1.convertToFormat(QImage::Format_ARGB32_Premultiplied);
  EXPECT_TRUE(overlay_image.hasAlphaChannel());
  setImageAlphaPremultiplied(&overlay_image, overlay_alpha);
//  overlay_image.fill(Qt::transparent);
  const std::string overlay1_output_filename = "overlay1.png";
  overlay_image.save(QString::fromStdString(overlay1_output_filename));
  QPainter painter(&overlay_image);
//  painter.setCompositionMode(QPainter::CompositionMode_SourceOver);
  painter.setCompositionMode(QPainter::CompositionMode_DestinationOver);
//  painter.drawImage(overlay_image.rect(), image1, image1.rect());
  painter.drawImage(overlay_image.rect(), image2, image2.rect());
  painter.end();
  const std::string overlay_output_filename = "overlay.png";
  overlay_image.save(QString::fromStdString(overlay_output_filename));
}

int main(int argc, char** argv) {
  QCoreApplication qapp(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
