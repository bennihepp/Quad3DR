//==================================================
// drawing_qt.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 05.04.17
//==================================================
#pragma once

#include <vector>
#include <bh/color.h>
#include <QImage>
#include <QPen>
#include <QPainter>
#include <bh/color.h>
#include "types.h"
#include "geometry.h"

namespace bh {
namespace vision {

template <typename FloatT>
void drawKeypoint(
        QImage* img,
        const Keypoint<FloatT>& keypoint,
        const FloatT point_size = FloatT(2),
        const QColor color = Qt::red) {
  QPainter painter(img);
  QPen pen;
  pen.setColor(color);
  pen.setWidth(0.2);
  painter.setPen(pen);
  const QPointF qt_point(keypoint.x(), keypoint.y());
  if (qt_point.x() >= 0 && qt_point.x() < img->width()
      && qt_point.y() >= 0 && qt_point.y() < img->height()) {
    painter.drawEllipse(qt_point, point_size, point_size);
  }
  painter.end();
}

template <typename FloatT>
void drawKeypoints(
        QImage* img,
        const std::vector<Keypoint<FloatT>>& keypoints,
        const FloatT point_size = FloatT(2)) {
  QPainter painter(img);
  QPen pen;
  pen.setColor(Qt::red);
  pen.setWidth(0.2);
  painter.setPen(pen);
  for (const Keypoint<FloatT>& keypoint : keypoints) {
    const QPointF qt_point(keypoint.x(), keypoint.y());
    if (qt_point.x() >= 0 && qt_point.x() < img->width()
      && qt_point.y() >= 0 && qt_point.y() < img->height()) {
      painter.drawEllipse(qt_point, point_size, point_size);
    }
  }
  painter.end();
}

template <typename FloatT>
QImage drawKeypoints(
        const size_t width, const size_t height,
        const std::vector<Keypoint<FloatT>>& keypoints,
        const FloatT point_size = FloatT(2)) {
  QImage img(width, height, QImage::Format_RGBA8888);
  drawKeypoints(&img, keypoints, point_size);
  return img;
}

template <typename FloatT>
void dumpKeypoints(
        const std::string& filename,
        const size_t width, const size_t height,
        const std::vector<Keypoint<FloatT>>& keypoints,
        const FloatT point_size = FloatT(2)) {
  const QImage img = drawKeypoints(width, height, keypoints, point_size);
  img.save(QString::fromStdString(filename));
}

template <typename FloatT>
void drawKeypointMatches(
        QImage* img,
        const size_t keypoint2_x_offset,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const bool draw_lines = true,
        const FloatT match_line_width = FloatT(0.5)) {
  BH_ASSERT(keypoints1.size() == keypoints2.size());
  QPainter painter(img);
  painter.setRenderHint(QPainter::Antialiasing);
  QPen pen;
  pen.setWidth(match_line_width);
  pen.setColor(Qt::green);
  painter.setPen(pen);
  for (size_t i = 0; i < keypoints1.size(); ++i) {
    const Keypoint<FloatT>& keypoint1 = keypoints1[i];
    const Keypoint<FloatT>& keypoint2 = keypoints2[i];
    const QPointF qt_point1(keypoint1.x(), keypoint1.y());
    const QPointF qt_point2(keypoint2.x(), keypoint2.y());
    const QLineF qt_line(qt_point1, qt_point2 + QPointF(keypoint2_x_offset, 0));
    painter.drawLine(qt_line);
  }
  painter.end();
}

template <typename FloatT>
void drawKeypointMatches(
        QImage* img,
        const size_t keypoint2_x_offset,
        const std::vector<KeypointMatch<FloatT>>& keypoint_matches,
        const bool draw_lines = true,
        const FloatT match_line_width = FloatT(0.5)) {
  QPainter painter(img);
  painter.setRenderHint(QPainter::Antialiasing);
  QPen pen;
  pen.setWidth(match_line_width);
  pen.setColor(Qt::green);
  painter.setPen(pen);
  for (size_t i = 0; i < keypoint_matches.size(); ++i) {
    const Keypoint<FloatT>& keypoint1 = keypoint_matches[i].keypoint1();
    const Keypoint<FloatT>& keypoint2 = keypoint_matches[i].keypoint2();
    const QPointF qt_point1(keypoint1.x(), keypoint1.y());
    const QPointF qt_point2(keypoint2.x(), keypoint2.y());
    const QLineF qt_line(qt_point1, qt_point2 + QPointF(keypoint2_x_offset, 0));
    painter.drawLine(qt_line);
  }
  painter.end();
}

template <typename FloatT>
void drawKeypointMatches(
        const size_t width, const size_t height,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const bool draw_lines = true,
        const FloatT match_line_width = FloatT(0.5)) {
  QImage img(2 * width, height, QImage::Format_RGBA8888);
  drawKeypointMatches(&img, width, keypoints1, keypoints2, draw_lines, match_line_width);
  return img;
}

template <typename FloatT>
void drawKeypointMatches(
        const size_t width, const size_t height,
        const std::vector<KeypointMatch<FloatT>>& keypoint_matches,
      const bool draw_lines = true,
      const FloatT match_line_width = FloatT(0.5)) {
  QImage img(2 * width, height, QImage::Format_RGBA8888);
  drawKeypointMatches(&img, width, keypoint_matches, draw_lines, match_line_width);
  return img;
}

template <typename FloatT>
QImage drawKeypointsAndMatches(
        const QImage& img1,
        const QImage& img2,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const bool draw_lines = true,
        const FloatT point_size = FloatT(2),
        const FloatT match_line_width = FloatT(0.5)) {
  BH_ASSERT(img1.height() == img2.height());
  QImage tmp_img1 = img1.copy();
  drawKeypoints(&tmp_img1, keypoints1, point_size);
  QImage tmp_img2 = img2.copy();
  drawKeypoints(&tmp_img2, keypoints2, point_size);
  QImage img(tmp_img1.width() + tmp_img2.width(), tmp_img1.height(), QImage::Format_RGBA8888);
  QPainter painter(&img);
  painter.drawImage(0, 0, tmp_img1);
  painter.drawImage(tmp_img1.width(), 0, tmp_img2);
  painter.end();
  drawKeypointMatches(&img, tmp_img1.width(), keypoints1, keypoints2, draw_lines, match_line_width);
  return img;
}

template <typename FloatT>
QImage drawKeypointsAndMatches(
        const QImage& img1,
        const QImage& img2,
        const std::vector<KeypointMatch<FloatT>>& keypoint_matches,
        const bool draw_lines = true,
        const FloatT point_size = FloatT(2),
        const FloatT match_line_width = FloatT(0.5)) {
  BH_ASSERT(img1.height() == img2.height());
  std::vector<Keypoint<FloatT>> keypoints1;
  std::transform(keypoint_matches.begin(), keypoint_matches.end(), std::back_inserter(keypoints1),
    [&](const KeypointMatch<FloatT>& keypoint_match) {
      return keypoint_match.keypoint1();
  });
  std::vector<Keypoint<FloatT>> keypoints2;
  std::transform(keypoint_matches.begin(), keypoint_matches.end(), std::back_inserter(keypoints2),
    [&](const KeypointMatch<FloatT>& keypoint_match) {
     return keypoint_match.keypoint2();
  });
  QImage tmp_img1 = img1.copy();
  drawKeypoints(&tmp_img1, keypoints1, point_size);
  QImage tmp_img2 = img2.copy();
  drawKeypoints(&tmp_img2, keypoints2, point_size);
  QImage img(tmp_img1.width() + tmp_img2.width(), tmp_img1.height(), QImage::Format_RGBA8888);
  QPainter painter(&img);
  painter.drawImage(0, 0, tmp_img1);
  painter.drawImage(tmp_img1.width(), 0, tmp_img2);
  painter.end();
  drawKeypointMatches(&img, tmp_img1.width(), keypoint_matches, draw_lines, match_line_width);
  return img;
}

template <typename FloatT>
QImage drawKeypointsAndMatches(
        const size_t width, const size_t height,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const bool draw_lines = true,
        const FloatT point_size = FloatT(2),
        const FloatT match_line_width = FloatT(0.5)) {
  QImage img1 = drawKeypoints(width, height, keypoints1, point_size);
  QImage img2 = drawKeypoints(width, height, keypoints2, point_size);
  QImage img(2 * width, height, QImage::Format_RGBA8888);
  QPainter painter(&img);
  painter.drawImage(0, 0, img1);
  painter.drawImage(img1.width(), 0, img2);
  painter.end();
  drawKeypointMatches(&img, width, keypoints1, keypoints2, draw_lines, match_line_width);
  return img;
}

template <typename FloatT>
void drawKeypointsAndMatches(
        const size_t width, const size_t height,
        const std::vector<KeypointMatch<FloatT>>& keypoint_matches,
        const bool draw_lines = true,
        const FloatT point_size = FloatT(2),
        const FloatT match_line_width = FloatT(0.5)) {
  std::vector<Keypoint<FloatT>> keypoints1;
  std::transform(keypoint_matches.begin(), keypoint_matches.end(), std::back_inserter(keypoints1),
    [&](const KeypointMatch<FloatT>& keypoint_match) {
      return keypoint_match.keypoint1();
  });
  std::vector<Keypoint<FloatT>> keypoints2;
  std::transform(keypoint_matches.begin(), keypoint_matches.end(), std::back_inserter(keypoints2),
    [&](const KeypointMatch<FloatT>& keypoint_match) {
      return keypoint_match.keypoint2();
  });
  QImage img1 = drawKeypoints(width, height, keypoints1, point_size);
  QImage img2 = drawKeypoints(width, height, keypoints2, point_size);
  QImage img(2 * width, height, QImage::Format_RGBA8888);
  QPainter painter(&img);
  painter.drawImage(0, 0, img1);
  painter.drawImage(img1.width(), 0, img2);
  painter.end();
  drawKeypointMatches(&img, width, keypoint_matches, draw_lines, match_line_width);
  return img;
}

template <typename FloatT>
void dumpKeypointsAndMatches(
        const std::string& filename,
        const QImage& img1,
        const QImage& img2,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const bool draw_lines = true,
        const FloatT point_size = FloatT(2),
        const FloatT match_line_width = FloatT(0.5)) {
  const QImage img = drawKeypointAndMatchess(img1, img2, keypoints1, keypoints2, draw_lines, point_size, point_size);
  img.save(QString::fromStdString(filename));
}

template <typename FloatT>
void dumpKeypointsAndMatches(
        const std::string& filename,
        const QImage& img1,
        const QImage& img2,
        const std::vector<KeypointMatch<FloatT>>& keypoint_matches,
        const bool draw_lines = true,
        const FloatT point_size = FloatT(2),
        const FloatT match_line_width = FloatT(0.5)) {
  const QImage img = drawKeypointAndMatchess(img1, img2, keypoint_matches, draw_lines, point_size, point_size);
  img.save(QString::fromStdString(filename));
}

template <typename FloatT>
void dumpKeypointsAndMatches(
        const std::string& filename,
        const size_t width, const size_t height,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const bool draw_lines = true,
        const FloatT point_size = FloatT(2),
        const FloatT match_line_width = FloatT(0.5)) {
  const QImage img = drawKeypointAndMatchess(width, height, keypoints1, keypoints2, draw_lines, point_size, point_size);
  img.save(QString::fromStdString(filename));
}

template <typename FloatT>
void dumpKeypointsAndMatches(
        const std::string& filename,
        const size_t width, const size_t height,
        const std::vector<KeypointMatch<FloatT>>& keypoint_matches,
        const bool draw_lines = true,
        const FloatT point_size = FloatT(2),
        const FloatT match_line_width = FloatT(0.5)) {
  const QImage img = drawKeypointAndMatchess(width, height, keypoint_matches, draw_lines, point_size, point_size);
  img.save(QString::fromStdString(filename));
}

template <typename FloatT>
void drawLine(
        QImage* img,
        const typename EigenTypes<FloatT>::Vector3& line,
        const FloatT line_width = FloatT(0.5),
        const QColor color = Qt::green) {
  QPainter painter(img);
  painter.setRenderHint(QPainter::Antialiasing);
  QPen pen;
  pen.setWidth(line_width);
  pen.setColor(color);
  painter.setPen(pen);
//  const QPointF qt_point1(0, img->height() - (-line(2) / line(1)));
//  const QPointF qt_point2(img->width(), img->height() - (-line(2) - line(0) * img->width()) / line(1));
  const QPointF qt_point1(0, (-line(2) / line(1)));
  const QPointF qt_point2(img->width(), (-line(2) - line(0) * img->width()) / line(1));
  const QLineF qt_line(qt_point1, qt_point2);
  painter.drawLine(qt_line);
  painter.end();
}

template <typename FloatT>
void drawEpipolarLineInLeftImage(
        QImage* left_img,
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const Keypoint<FloatT>& right_keypoint,
        const FloatT epipolar_line_width = FloatT(0.5),
        const QColor color = Qt::green) {
  using Vector2 = typename EigenTypes<FloatT>::Vector2;
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  const Vector2 right_image_point(right_keypoint.x(), right_keypoint.y());
  const Vector3 epipolar_line = fundamental_matrix.transpose() * right_image_point.homogeneous();
  drawLine(left_img, epipolar_line, epipolar_line_width, color);
}

template <typename FloatT>
void drawEpipolarLineInRightImage(
        QImage* right_img,
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const Keypoint<FloatT>& left_keypoint,
        const FloatT epipolar_line_width = FloatT(0.5),
        const QColor color = Qt::green) {
  using Vector2 = typename EigenTypes<FloatT>::Vector2;
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  const Vector2 left_image_point(left_keypoint.x(), left_keypoint.y());
  const Vector3 epipolar_line = fundamental_matrix * left_image_point.homogeneous();
  drawLine(right_img, epipolar_line, epipolar_line_width, color);
}

template <typename FloatT>
void drawEpipolarLinesInPlace(
        QImage* img1,
        QImage* img2,
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const FloatT epipolar_line_width = FloatT(0.5)) {
  using Vector2 = typename EigenTypes<FloatT>::Vector2;
  using Vector3 = typename EigenTypes<FloatT>::Vector3;
  using Keypoint = Keypoint<FloatT>;
  for (const Keypoint& keypoint1 : keypoints1) {
    const Vector2 image_point1(keypoint1.x(), keypoint1.y());
    const Vector3 epipolar_line = fundamental_matrix * image_point1.homogeneous();
    drawLine(img2, epipolar_line, epipolar_line_width, Qt::green);
  }
  for (const Keypoint& keypoint1 : keypoints1) {
    const Vector2 image_point1(keypoint1.x(), keypoint1.y());
    const Vector3 epipolar_line = (image_point1.homogeneous().transpose() * fundamental_matrix).transpose();
    drawLine(img2, epipolar_line, epipolar_line_width, Qt::yellow);
  }
  for (const Keypoint& keypoint2 : keypoints2) {
    const Vector2 image_point2(keypoint2.x(), keypoint2.y());
    const Vector3 epipolar_line = (image_point2.homogeneous().transpose() * fundamental_matrix).transpose();
    drawLine(img1, epipolar_line, epipolar_line_width, Qt::green);
  }
  for (const Keypoint& keypoint2 : keypoints2) {
    const Vector2 image_point2(keypoint2.x(), keypoint2.y());
    const Vector3 epipolar_line = fundamental_matrix * image_point2.homogeneous();
    drawLine(img1, epipolar_line, epipolar_line_width, Qt::yellow);
  }
}

template <typename FloatT>
QImage drawEpipolarLines(
        const QImage& img1,
        const QImage& img2,
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const FloatT epipolar_line_width = FloatT(0.5)) {
  QImage tmp_img1 = img1.copy();
  QImage tmp_img2 = img2.copy();
  drawEpipolarLinesInPlace(&tmp_img1, &tmp_img2, fundamental_matrix, keypoints1, keypoints2, epipolar_line_width);
  QImage img(tmp_img1.width() + tmp_img2.width(), tmp_img1.height(), QImage::Format_RGBA8888);
  QPainter painter(&img);
  painter.drawImage(0, 0, tmp_img1);
  painter.drawImage(tmp_img1.width(), 0, tmp_img2);
  painter.end();
  return img;
}

template <typename FloatT>
QImage drawKeypointsAndEpipolarLines(
        const QImage& img1,
        const QImage& img2,
        const FundamentalMatrix<FloatT>& fundamental_matrix,
        const std::vector<Keypoint<FloatT>>& keypoints1,
        const std::vector<Keypoint<FloatT>>& keypoints2,
        const FloatT point_size = FloatT(5),
        const FloatT epipolar_line_width = FloatT(0.5)) {
  using Keypoint = Keypoint<FloatT>;
  BH_ASSERT(img1.height() == img2.height());
  QImage tmp_img1 = img1.copy();
  drawKeypoints(&tmp_img1, keypoints1, point_size);
  QImage tmp_img2 = img2.copy();
  bh::ColorMapJet<FloatT> cmap;
  for (size_t i = 0; i < keypoints1.size(); ++i) {
    const Keypoint& keypoint1 = keypoints1[i];
    const Keypoint& keypoint2 = keypoints2[i];
    const bh::Color3<FloatT> color = cmap.map(bh::normalize<FloatT>(i, 0, keypoints1.size() - 1));
    const QColor qt_color = QColor::fromRgbF(color.r(), color.g(), color.b());
    drawKeypoint(&tmp_img2, keypoint2, point_size, qt_color);
    drawKeypoint(&tmp_img1, keypoint1, point_size, qt_color);
    drawEpipolarLineInLeftImage(&tmp_img1, fundamental_matrix, keypoint2, epipolar_line_width, qt_color);
    drawEpipolarLineInRightImage(&tmp_img2, fundamental_matrix, keypoint1, epipolar_line_width, qt_color);
  }
  QImage img(tmp_img1.width() + tmp_img2.width(), tmp_img1.height(), QImage::Format_RGBA8888);
  QPainter painter(&img);
  painter.drawImage(0, 0, tmp_img1);
  painter.drawImage(tmp_img1.width(), 0, tmp_img2);
  painter.end();
  return img;
}

}
}
