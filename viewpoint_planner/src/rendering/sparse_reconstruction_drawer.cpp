//==================================================
// sparse_reconstruction_drawer.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 16, 2016
//==================================================

#include "sparse_reconstruction_drawer.h"
#include <array>
#include <iostream>
#include <QtOpenGL>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <bh/common.h>
#include <bh/utilities.h>

namespace rendering {

SparseReconstructionDrawer::SparseReconstructionDrawer()
: sparse_recon_(nullptr), camera_size_(0.5f), point_size_(1.0f), draw_cameras_(true), draw_sparse_points_(true) {}

SparseReconstructionDrawer::~SparseReconstructionDrawer() {
  clear();
}

void SparseReconstructionDrawer::setSparseReconstruction(const SparseReconstruction* sparse_recon) {
  sparse_recon_ = sparse_recon;
  upload();
}

void SparseReconstructionDrawer::changeCameraSize(const float delta) {
  if (delta == 0.0f) {
    return;
  }
  camera_size_ *= (1.0f + delta / 100.0f * CAMERA_SIZE_SPEED);
  camera_size_ = bh::clamp(camera_size_, MIN_CAMERA_SIZE, MAX_CAMERA_SIZE);
  uploadCameraData();
}

void SparseReconstructionDrawer::changePointSize(const float delta) {
  if (delta == 0.0f) {
    return;
  }
  point_size_ *= (1.0f + delta / 100.0f * POINT_SIZE_SPEED);
  point_size_ = bh::clamp(point_size_, MIN_POINT_SIZE, MAX_POINT_SIZE);
  uploadPointData();
}

void SparseReconstructionDrawer::setCameraSize(float camera_size) {
  camera_size_ = camera_size;
  uploadCameraData();
}

void SparseReconstructionDrawer::setDrawCameras(bool draw_cameras) {
  draw_cameras_ =  draw_cameras;
}

void SparseReconstructionDrawer::setDrawSparsePoints(bool draw_sparse_points) {
  draw_sparse_points_ = draw_sparse_points;
}

void SparseReconstructionDrawer::clear() {
  camera_triangle_drawer_.clear();
  camera_line_drawer_.clear();
  sparse_point_drawer_.clear();
}

void SparseReconstructionDrawer::init() {
  camera_triangle_drawer_.init();
  camera_line_drawer_.init();
  sparse_point_drawer_.init();
  upload();
}

void SparseReconstructionDrawer::upload() {
  uploadCameraData();
  uploadPointData();
}

void SparseReconstructionDrawer::draw(const QMatrix4x4& pvm_matrix, const int width, const int height) {
  if (draw_cameras_) {
    camera_triangle_drawer_.draw(pvm_matrix);
    camera_line_drawer_.draw(pvm_matrix, width, height, CAMERA_LINE_WIDTH);
  }
  if (draw_sparse_points_) {
    sparse_point_drawer_.draw(pvm_matrix, point_size_);
  }
}

void SparseReconstructionDrawer::uploadCameraData() {
  if (sparse_recon_ == nullptr) {
    return;
  }
  const SparseReconstruction::CameraMapType& cameras = sparse_recon_->getCameras();
  const SparseReconstruction::ImageMapType& images = sparse_recon_->getImages();
  std::vector<OGLTriangleData> triangle_data;
  triangle_data.reserve(2 * images.size());
  std::vector<OGLLineData> line_data;
  line_data.reserve(8 * images.size());

  for (const auto& entry : images) {
    const ImageColmap& image = entry.second;
    const PinholeCameraColmap& camera = cameras.at(image.camera_id());

    float r, g, b, a;
    r = IMAGE_R;
    g = IMAGE_G;
    b = IMAGE_B;
    a = IMAGE_A;

    std::array<OGLLineData, 8> lines;
    std::array<OGLTriangleData, 2> triangles;
    generateImageModel(camera, image, camera_size_, r, g, b, a, lines, triangles);

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

void SparseReconstructionDrawer::uploadPointData() {
  if (sparse_recon_ == nullptr) {
    return;
  }
  const SparseReconstruction::Point3DMapType& points3D = sparse_recon_->getPoints3D();
  std::vector<OGLVertexDataRGBA> point_data;
  point_data.reserve(points3D.size());

  for (const auto& entry : points3D) {
    const reconstruction::Point3D& point3D = entry.second;
    if (point3D.error <= RENDER_MAX_POINT_ERROR
        && point3D.feature_track.size() >= RENDER_MIN_TRACK_LENGTH) {
      OGLVertexDataRGBA point;
      point.x = static_cast<float>(point3D.pos(0));
      point.y = static_cast<float>(point3D.pos(1));
      point.z = static_cast<float>(point3D.pos(2));
      point.r = point3D.color.r() / 255.f;
      point.g = point3D.color.g() / 255.f;
      point.b = point3D.color.b() / 255.f;
      point.a = 1;
      point_data.push_back(point);
    }
  }

  sparse_point_drawer_.upload(point_data);
}

void SparseReconstructionDrawer::generateImageModel(const PinholeCameraColmap& camera, const ImageColmap& image,
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

  const Eigen::Matrix<float, 3, 4> transform_image_to_world =
      image.pose().getTransformationImageToWorld().cast<float>();
//        std::cout << "inv_proj_matrix=" << inv_proj_matrix << std::endl;

  // Projection center, top-left, top-right, bottom-right, bottom-left corners
  const Eigen::Vector3f pc = transform_image_to_world.rightCols<1>().cast<float>();
  const Eigen::Vector3f tl = transform_image_to_world
                             * Eigen::Vector4f(-image_width, image_height, focal_length, 1);
  const Eigen::Vector3f tr = transform_image_to_world
                             * Eigen::Vector4f(image_width, image_height, focal_length, 1);
  const Eigen::Vector3f br = transform_image_to_world
                             * Eigen::Vector4f(image_width, -image_height, focal_length, 1);
  const Eigen::Vector3f bl = transform_image_to_world
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

}
