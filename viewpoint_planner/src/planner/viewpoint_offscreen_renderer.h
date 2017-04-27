//==================================================
// viewpoint_offscreen_renderer.h.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 28.03.17
//

#pragma once

#include <bh/color.h>
#include <mutex>
#include "viewpoint_planner_types.h"
#include "viewpoint.h"

#if WITH_OPENGL_OFFSCREEN
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QOpenGLFunctions>
#include <QSurfaceFormat>
#include <bh/qt/utils.h>
#include <bh/color.h>
#include <bh/mLib/mLib.h>
#include <bh/config_options.h>
#include "../rendering/triangle_drawer.h"
#endif

namespace viewpoint_planner {

class ViewpointOffscreenRenderer {
public:
  using MeshType = ml::MeshData<FloatType>;

  struct Options : bh::ConfigOptions {
    Options() {
      addOption<bool>("dump_poisson_mesh_normals_image", &dump_poisson_mesh_normals_image);
      addOption<bool>("dump_poisson_mesh_depth_image", &dump_poisson_mesh_depth_image);
    }

    ~Options() override {}

    // Whether to dump the poisson mesh normals image after rendering
    bool dump_poisson_mesh_normals_image = false;
    // Whether to dump the poisson mesh depth image after rendering
    bool dump_poisson_mesh_depth_image = false;
  };

  explicit ViewpointOffscreenRenderer(const PinholeCamera& camera, const MeshType* poisson_mesh);

  explicit ViewpointOffscreenRenderer(const Options& options, const PinholeCamera& camera, const MeshType* poisson_mesh);

  virtual ~ViewpointOffscreenRenderer();

  bool isInitialized() const;

  void setClearColor(const bh::Color4<FloatType>& color);

  void setAntialiasing(const bool antialiasing);

  void setCamera(const PinholeCamera& camera);

  qreal getNearPlane() const;

  qreal getFarPlane() const;

  void setNearFarPlane(const qreal near_plane, const qreal far_plane);

  std::unique_lock<std::mutex> acquireOpenGLLock() const;

  void initializeOpenGL() const;

  void clearOpenGL() const;

  void uploadPoissonMesh() const;

  void bindOpenGLFbo() const;

  void releaseOpenGLFbo() const;

  void beginOpenGLDrawing() const;

  void finishOpenGLDrawing() const;

  void clear() const;

  void clear(const bh::Color4<FloatType>& color) const;

  void clearWithoutLock() const;

  void clearWithoutLock(const bh::Color4<FloatType>& color) const;

  QMatrix4x4 getPvmMatrixFromViewpoint(const Viewpoint& viewpoint) const;

  QMatrix4x4 getPvmMatrixFromPose(const Pose& pose) const;

  QMatrix4x4 getVmMatrixFromViewpoint(const Viewpoint& viewpoint) const;

  QMatrix4x4 getVmMatrixFromPose(const Pose& pose) const;

  QImage drawPoissonMesh(const Viewpoint& viewpoint, const bool clear_viewport = true) const;

  QImage drawPoissonMesh(const Pose& pose, const bool clear_viewport = true) const;

  QImage drawPoissonMesh(const QMatrix4x4& pvm_matrix, const bool clear_viewport = true) const;

  QImage drawPoissonMeshNormals(const Viewpoint& viewpoint, const bool clear_viewport = true) const;

  QImage drawPoissonMeshNormals(const Pose& pose, const bool clear_viewport = true) const;

  QImage drawPoissonMeshNormals(const QMatrix4x4& pvm_matrix, const bool clear_viewport = true) const;

  QImage drawPoissonMeshDepth(const Viewpoint& viewpoint, const bool clear_viewport = true) const;

  QImage drawPoissonMeshDepth(const Pose& pose, const bool clear_viewport = true) const;

  QImage drawPoissonMeshDepth(
          const QMatrix4x4& pvm_matrix, const QMatrix4x4& vm_matrix, const bool clear_viewport = true) const;

  QImage drawPoissonMeshIndices(const Viewpoint& viewpoint, const bool clear_viewport = true) const;

  QImage drawPoissonMeshIndices(const Pose& pose, const bool clear_viewport = true) const;

  QImage drawPoissonMeshIndices(const QMatrix4x4& pvm_matrix, const bool clear_viewport = true) const;

  Vector3 computePoissonMeshNormalVector(
          const Viewpoint& viewpoint, const Vector3& position) const;

  Vector3 computePoissonMeshNormalVector(
          const Viewpoint& viewpoint,
          const std::size_t x, const std::size_t y) const;

  FloatType computePoissonMeshDepth(
          const Viewpoint& viewpoint, const Vector3& position) const;

  FloatType computePoissonMeshDepth(
          const Viewpoint& viewpoint,
          const std::size_t x, const std::size_t y) const;

  bh::Color4<uint8_t> encodeDepthValue(const FloatType depth) const;

  FloatType decodeDepthValue(const QImage& depth_image, const Vector2& image_point) const;

  FloatType decodeDepthValue(const bh::Color4<uint8_t>& color) const;

  FloatType decodeDepthValue(const QColor& pixel) const;

  QImage convertEncodedDepthImageToRGB(
          const QImage& encoded_image, const FloatType min_depth, const FloatType max_depth) const;

  QImage convertEncodedIndicesImageToRGB(
          const QImage& encoded_image, const size_t min_index, const size_t max_index) const;

  std::unordered_set<size_t> getVisibleTriangles(const QImage& mesh_indices_image) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Options options_;

  mutable std::mutex opengl_mutex_;
  mutable std::mutex poisson_mesh_cache_mutex_;

  mutable QOpenGLContext* opengl_context_;
  mutable QOffscreenSurface* opengl_surface_;
  mutable QOpenGLFramebufferObject* opengl_fbo_;
  mutable rendering::TriangleDrawer* poisson_mesh_drawer_;

  bool antialiasing_;
  bh::Color4<FloatType> clear_color_;

  mutable Pose cached_poisson_mesh_normals_pose_;
  mutable QImage cached_poisson_mesh_normals_image_;
  mutable Pose cached_poisson_mesh_depth_pose_;
  mutable QImage cached_poisson_mesh_depth_image_;

  PinholeCamera camera_;
  qreal near_plane_;
  qreal far_plane_;

  const MeshType* poisson_mesh_;
};

}