//==================================================
// viewpoint_planner_dump.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 8, 2017
//==================================================

#include "viewpoint_planner.h"

#pragma GCC push_options
#pragma GCC optimize("O0")
QImage ViewpointPlanner::drawSparsePoints(
    const Viewpoint& viewpoint,
    const FloatType sparse_point_size /*= FloatType(2)*/) const {
  const std::unordered_set<Point3DId>& sparse_points_visible = computeVisibleSparsePoints(viewpoint);
  QImage img = drawPoissonMesh(viewpoint);
  QPainter painter(&img);
  QPen pen;
  pen.setColor(Qt::red);
  pen.setWidth(0.2);
  painter.setPen(pen);
  for (const Point3DId& point3d_id: sparse_points_visible) {
    const Point3D& point3d = data_->reconstruction_->getPoints3D().at(point3d_id);
    if (viewpoint.isWorldPointVisible(point3d.getPosition())) {
      const Vector2 point_image = viewpoint.projectWorldPointIntoImage(point3d.getPosition());
      painter.drawEllipse(QPointF(point_image(0), point_image(1)), sparse_point_size, sparse_point_size);
    }
  }
  painter.end();
  return img;
}

QImage ViewpointPlanner::drawSparsePoints(
    const ViewpointEntryIndex viewpoint_index,
    const FloatType sparse_point_size /*= FloatType(2)*/) const {
  const Viewpoint& viewpoint = viewpoint_entries_[viewpoint_index].viewpoint;
  const std::unordered_set<Point3DId>& sparse_points_visible = getCachedVisibleSparsePoints(viewpoint_index);
  QImage img = drawPoissonMesh(viewpoint_index);
  QPainter painter(&img);
  QPen pen;
  pen.setColor(Qt::red);
  pen.setWidth(0.2);
  painter.setPen(pen);
  for (const Point3DId& point3d_id: sparse_points_visible) {
    const Point3D& point3d = data_->reconstruction_->getPoints3D().at(point3d_id);
    if (viewpoint.isWorldPointVisible(point3d.getPosition())) {
      const Vector2 point_image = viewpoint.projectWorldPointIntoImage(point3d.getPosition());
      painter.drawEllipse(QPointF(point_image(0), point_image(1)), sparse_point_size, sparse_point_size);
    }
  }
  painter.end();
  return img;
}

void ViewpointPlanner::dumpSparsePoints(const Viewpoint& viewpoint,
                                             const std::string& filename,
                           const FloatType sparse_point_size /*= FloatType(2)*/) const {
  const QImage img = drawSparsePoints(viewpoint, sparse_point_size);
  img.save(QString::fromStdString(filename));
}

void ViewpointPlanner::dumpSparsePoints(const ViewpointEntryIndex viewpoint_index,
                                             const std::string& filename,
                                             const FloatType sparse_point_size /*= FloatType(2)*/) const {
  const QImage img = drawSparsePoints(viewpoint_index, sparse_point_size);
  img.save(QString::fromStdString(filename));
}

void ViewpointPlanner::dumpSparsePoints(const ViewpointEntryIndex viewpoint_index,
                                             const FloatType sparse_point_size /*= FloatType(2)*/) const {
  const QImage img = drawSparsePoints(viewpoint_index, sparse_point_size);
  if (!boost::filesystem::is_directory("dump")) {
    boost::filesystem::create_directories("dump");
  }
  dumpSparsePoints(
      viewpoint_index,
      std::string("dump/sparse_points_") + std::to_string(viewpoint_index) + ".png",
          sparse_point_size);
}

QImage ViewpointPlanner::drawSparseMatching(
    const Viewpoint& viewpoint1, const Viewpoint& viewpoint2,
    const bool draw_lines /*= true*/,
    const FloatType sparse_point_size /*= FloatType(2)*/, const FloatType match_line_width /*= FloatType(0.5)*/) const {
  const std::unordered_set<Point3DId>& sparse_points_visible1 = computeVisibleSparsePoints(viewpoint1);
  const std::unordered_set<Point3DId>& sparse_points_visible2 = computeVisibleSparsePoints(viewpoint2);
  const QImage img1 = drawSparsePoints(viewpoint1, sparse_point_size);
  const QImage img2 = drawSparsePoints(viewpoint2, sparse_point_size);
  AIT_ASSERT(img1.height() == img2.height());
  QImage img(img1.width() + img2.width(), img1.height(), QImage::Format_ARGB32);
  QPainter painter(&img);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.drawImage(0, 0, img1);
  painter.drawImage(img1.width(), 0, img2);
  QPen pen_matchable;
  pen_matchable.setWidth(match_line_width);
  pen_matchable.setColor(Qt::green);
  QPen pen_not_matchable;
  pen_not_matchable.setWidth(match_line_width);
  pen_not_matchable.setColor(Qt::yellow);
  if (draw_lines) {
    for (const Point3DId& point3d_id: sparse_points_visible1) {
      if (sparse_points_visible2.count(point3d_id) > 0) {
        const Point3D& point3d = data_->reconstruction_->getPoints3D().at(point3d_id);
        const bool matchable = isSparsePointMatchable(viewpoint1, viewpoint2, point3d);
        if (matchable) {
          painter.setPen(pen_matchable);
        }
        else {
          painter.setPen(pen_not_matchable);
        }
        const Vector2 image_point1 = viewpoint1.projectWorldPointIntoImage(point3d.getPosition());
        const Vector2 image_point2 = viewpoint2.projectWorldPointIntoImage(point3d.getPosition());
        const QLineF line(image_point1(0), image_point1(1),
               img1.width() + image_point2(0), image_point2(1));
        painter.drawLine(line);
      }
    }
  }
  painter.end();
  return img;
}

QImage ViewpointPlanner::drawSparseMatching(
    const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2,
    const bool draw_lines /*= true*/,
    const FloatType sparse_point_size /*= FloatType(2)*/, const FloatType match_line_width /*= FloatType(0.5)*/) const {
  const std::unordered_set<Point3DId>& sparse_points_visible1 = getCachedVisibleSparsePoints(viewpoint_index1);
  const std::unordered_set<Point3DId>& sparse_points_visible2 = getCachedVisibleSparsePoints(viewpoint_index2);
  const Viewpoint& viewpoint1 = viewpoint_entries_[viewpoint_index1].viewpoint;
  const Viewpoint& viewpoint2 = viewpoint_entries_[viewpoint_index2].viewpoint;
  const QImage img1 = drawSparsePoints(viewpoint_index1, sparse_point_size);
  const QImage img2 = drawSparsePoints(viewpoint_index2, sparse_point_size);
  AIT_ASSERT(img1.height() == img2.height());
  QImage img(img1.width() + img2.width(), img1.height(), QImage::Format_ARGB32);
  QPainter painter(&img);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.drawImage(0, 0, img1);
  painter.drawImage(img1.width(), 0, img2);
  QPen pen_matchable;
  pen_matchable.setWidth(match_line_width);
  pen_matchable.setColor(Qt::green);
  QPen pen_not_matchable;
  pen_not_matchable.setWidth(match_line_width);
  pen_not_matchable.setColor(Qt::yellow);
  if (draw_lines) {
    for (const Point3DId& point3d_id: sparse_points_visible1) {
      if (sparse_points_visible2.count(point3d_id) > 0) {
        const Point3D& point3d = data_->reconstruction_->getPoints3D().at(point3d_id);
        const bool matchable = isSparsePointMatchable(viewpoint1, viewpoint2, point3d);
        if (matchable) {
          painter.setPen(pen_matchable);
        }
        else {
          painter.setPen(pen_not_matchable);
        }
        const Vector2 image_point1 = viewpoint1.projectWorldPointIntoImage(point3d.getPosition());
        const Vector2 image_point2 = viewpoint2.projectWorldPointIntoImage(point3d.getPosition());
        const QLineF line(image_point1(0), image_point1(1),
               img1.width() + image_point2(0), image_point2(1));
        painter.drawLine(line);
      }
    }
  }
  painter.end();
  return img;
}

void ViewpointPlanner::dumpSparseMatching(
    const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2,
    const std::string& filename,
    const bool draw_lines /*= true*/,
    const FloatType sparse_point_size /*= FloatType(2)*/, const FloatType match_line_width /*= FloatType(0.5)*/) const {
  const QImage img = drawSparseMatching(viewpoint_index1, viewpoint_index2, draw_lines, sparse_point_size, match_line_width);
  img.save(QString::fromStdString(filename));
}

void ViewpointPlanner::dumpSparseMatching(
    const ViewpointEntryIndex viewpoint_index1, const ViewpointEntryIndex viewpoint_index2,
    const bool draw_lines /*= true*/,
    const FloatType sparse_point_size /*= FloatType(2)*/, const FloatType match_line_width /*= FloatType(0.5)*/) const {
  if (!boost::filesystem::is_directory("dump")) {
    boost::filesystem::create_directories("dump");
  }
  dumpSparseMatching(
      viewpoint_index1, viewpoint_index2,
      std::string("dump/sparse_matching_")
      + std::to_string(viewpoint_index1) + "_" + std::to_string(viewpoint_index2) + ".png",
      draw_lines,
      sparse_point_size, match_line_width);
}

void ViewpointPlanner::dumpSparseMatching(
    const Viewpoint& viewpoint1, const Viewpoint& viewpoint2,
    const std::string& filename,
    const bool draw_lines /*= true*/,
    const FloatType sparse_point_size /*= FloatType(2)*/, const FloatType match_line_width /*= FloatType(0.5)*/) const {
  const QImage img = drawSparseMatching(viewpoint1, viewpoint2, draw_lines, sparse_point_size, match_line_width);
  img.save(QString::fromStdString(filename));
}
#pragma GCC pop_options
