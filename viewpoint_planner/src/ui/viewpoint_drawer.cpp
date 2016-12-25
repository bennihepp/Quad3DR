/*
 * viewpoint_drawer.cpp
 *
 *  Created on: Dec 24, 2016
 *      Author: bhepp
 */

#include "viewpoint_drawer.h"
#include <array>
#include <iostream>
#include <QtOpenGL>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <ait/common.h>
#include <ait/utilities.h>

ViewpointDrawer::ViewpointDrawer()
: camera_size_(0.5f), draw_cameras_(true) {}

ViewpointDrawer::~ViewpointDrawer() {
  clear();
}

void ViewpointDrawer::setCamera(const PinholeCamera& camera) {
  camera_ = camera;
}

void ViewpointDrawer::setViewpoints(const std::vector<ait::Pose>& poses) {
  poses_ = poses;
  upload();
}

void ViewpointDrawer::changeCameraSize(const float delta) {
  if (delta == 0.0f) {
    return;
  }
  camera_size_ *= (1.0f + delta / 100.0f * CAMERA_SIZE_SPEED);
  camera_size_ = ait::clamp(camera_size_, MIN_CAMERA_SIZE, MAX_CAMERA_SIZE);
  uploadCameraData();
}

void ViewpointDrawer::setCameraSize(float camera_size) {
  camera_size_ = camera_size;
  uploadCameraData();
}

void ViewpointDrawer::setDrawCameras(bool draw_cameras) {
  draw_cameras_ =  draw_cameras;
}

void ViewpointDrawer::clear() {
  camera_triangle_drawer_.clear();
  camera_line_drawer_.clear();
}

void ViewpointDrawer::init() {
  camera_triangle_drawer_.init();
  camera_line_drawer_.init();
  upload();
}

void ViewpointDrawer::upload() {
  uploadCameraData();
}

void ViewpointDrawer::draw(const QMatrix4x4& pvm_matrix, const int width, const int height) {
  if (draw_cameras_) {
    camera_triangle_drawer_.draw(pvm_matrix);
    camera_line_drawer_.draw(pvm_matrix, width, height, CAMERA_LINE_WIDTH);
  }
}

void ViewpointDrawer::uploadCameraData() {
  if (poses_.empty()) {
    return;
  }
  if (!camera_.isValid()) {
    return;
  }
  std::cout << "intrinsics: " << camera_.intrinsics() << std::endl;
  std::vector<OGLTriangleData> triangle_data;
  triangle_data.reserve(2 * poses_.size());
  std::vector<OGLLineData> line_data;
  line_data.reserve(8 * poses_.size());

  for (const ait::Pose& pose : poses_) {
    float r, g, b, a;
    r = IMAGE_R;
    g = IMAGE_G;
    b = IMAGE_B;
    a = IMAGE_A;

    std::array<OGLLineData, 8> lines;
    std::array<OGLTriangleData, 2> triangles;
    generateImageModel(camera_, pose, camera_size_, r, g, b, a, lines, triangles);

    for (const OGLLineData& line : lines) {
      line_data.push_back(line);
    }
    for (const OGLTriangleData& triangle : triangles) {
      triangle_data.push_back(triangle);
    }
  }

  camera_triangle_drawer_.upload(triangle_data);
  camera_line_drawer_.upload(line_data);
}

void ViewpointDrawer::generateImageModel(const PinholeCamera& camera, const ait::Pose& pose,
        const float camera_size, const float r, const float g, const float b, const float a,
        std::array<OGLLineData, 8>& lines, std::array<OGLTriangleData, 2>& triangles) {
  // Generate camera frustum in OpenGL coordinates
  const float image_width = camera_size * camera.width() / 1024.0f;
  const float image_height =
      image_width * static_cast<float>(camera.height()) / camera.width();
  const float image_extent = std::max(image_width, image_height);
  const float camera_extent = std::max(camera.width(), camera.height());
  const float camera_extent_normalized =
      static_cast<float>(camera_extent / camera.getMeanFocalLength());
  const float focal_length = 2.0f * image_extent / camera_extent_normalized;

  const Eigen::Matrix<float, 3, 4> inv_proj_matrix =
      pose.getTransformationImageToWorld().cast<float>();
//        std::cout << "inv_proj_matrix=" << inv_proj_matrix << std::endl;

  // Projection center, top-left, top-right, bottom-right, bottom-left corners
  const Eigen::Vector3f pc = inv_proj_matrix.rightCols<1>().cast<float>();
  const Eigen::Vector3f tl = inv_proj_matrix
                             * Eigen::Vector4f(-image_width, image_height, focal_length, 1);
  const Eigen::Vector3f tr = inv_proj_matrix
                             * Eigen::Vector4f(image_width, image_height, focal_length, 1);
  const Eigen::Vector3f br = inv_proj_matrix
                             * Eigen::Vector4f(image_width, -image_height, focal_length, 1);
  const Eigen::Vector3f bl = inv_proj_matrix
                             * Eigen::Vector4f(-image_width, -image_height, focal_length, 1);

//        std::cout << "pc=" << pc << std::endl;
//        std::cout << "tl=" << tl << std::endl;
//        std::cout << "tr=" << tr << std::endl;
//        std::cout << "br=" << br << std::endl;
//        std::cout << "bl=" << bl << std::endl;

  // Lines from sensor corners to projection center
  lines[0].vertex1 = OGLVertexDataRGBA(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
  lines[0].vertex2 = OGLVertexDataRGBA(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);

  lines[1].vertex1 = OGLVertexDataRGBA(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
  lines[1].vertex2 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);

  lines[2].vertex1 = OGLVertexDataRGBA(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
  lines[2].vertex2 = OGLVertexDataRGBA(br(0), br(1), br(2), 0.8f * r, g, b, 1);

  lines[3].vertex1 = OGLVertexDataRGBA(pc(0), pc(1), pc(2), 0.8f * r, g, b, 1);
  lines[3].vertex2 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);

  lines[4].vertex1 = OGLVertexDataRGBA(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);
  lines[4].vertex2 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);

  lines[5].vertex1 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), 0.8f * r, g, b, 1);
  lines[5].vertex2 = OGLVertexDataRGBA(br(0), br(1), br(2), 0.8f * r, g, b, 1);

  lines[6].vertex1 = OGLVertexDataRGBA(br(0), br(1), br(2), 0.8f * r, g, b, 1);
  lines[6].vertex2 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);

  lines[7].vertex1 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), 0.8f * r, g, b, 1);
  lines[7].vertex2 = OGLVertexDataRGBA(tl(0), tl(1), tl(2), 0.8f * r, g, b, 1);

  // Sensor rectangle
  triangles[0].vertex1 = OGLVertexDataRGBA(tl(0), tl(1), tl(2), r, g, b, a);
  triangles[0].vertex2 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), r, g, b, a);
  triangles[0].vertex3 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), r, g, b, a);

  triangles[1].vertex1 = OGLVertexDataRGBA(bl(0), bl(1), bl(2), r, g, b, a);
  triangles[1].vertex2 = OGLVertexDataRGBA(tr(0), tr(1), tr(2), r, g, b, a);
  triangles[1].vertex3 = OGLVertexDataRGBA(br(0), br(1), br(2), r, g, b, a);
}
