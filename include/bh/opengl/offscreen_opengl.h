//==================================================
// offscreen_opengl.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 19.04.17
//==================================================
#pragma once

#include <mutex>
#include <bh/color.h>
#include <bh/qt/utils.h>
#include <bh/vision/cameras.h>
#include <bh/pose.h>

#include <QImage>
#include <QThread>
#include <QPainter>
#include <QOpenGLContext>
#include <QOpenGLFramebufferObject>
#include <QOffscreenSurface>
#include <QOpenGLFunctions>
#include <QSurfaceFormat>

namespace bh {
namespace opengl {

template<typename FloatT>
class OffscreenOpenGL{
public:
  using FloatType = FloatT;
  BH_USE_FIXED_EIGEN_TYPES(FloatT);

  using Pose = bh::Pose<FloatT>;
  using Color4 = bh::Color4<FloatT>;
  using Camera = bh::vision::PinholeCamera<FloatT>;

  class DrawingHandle {
  public:
    DrawingHandle(const DrawingHandle &other) = delete;

    DrawingHandle(DrawingHandle &&other) = default;

    ~DrawingHandle();

    void finish();

    QImage getImageQt();

    const OffscreenOpenGL* operator->() const;

    OffscreenOpenGL* operator->();

  private:
    friend class OffscreenOpenGL;

    DrawingHandle(OffscreenOpenGL &opengl, const bool clear_viewport = true);

    bool finished_;
    OffscreenOpenGL &opengl_;
    std::unique_lock<std::mutex> lock_;
  };

  explicit OffscreenOpenGL(const Camera &camera);

  virtual ~OffscreenOpenGL();

  bool isInitialized() const;

  void setClearColor(const Color4& color);

  void setAntialiasing(const bool antialiasing);

  void setCamera(const Camera &camera);

  FloatT getNearPlane() const;

  FloatT getFarPlane() const;

  void setNearFarPlane(const FloatT near_plane, const FloatT far_plane);

  std::unique_lock<std::mutex> acquireOpenGLLock() const;

  const QOpenGLContext* getContext() const;

  QOpenGLContext* getContext();

  const QOffscreenSurface* getSurface() const;

  QOffscreenSurface* getSurface();

  const QOpenGLFramebufferObject* getFramebuffer() const;

  QOpenGLFramebufferObject* getFramebuffer();

  const QThread* getContextThread() const;

  void initializeOpenGL() const;

  void clearOpenGL() const;

  void bindOpenGLFbo() const;

  void releaseOpenGLFbo() const;

  void beginOpenGLDrawing() const;

  void finishOpenGLDrawing() const;

  DrawingHandle beginDrawing(const bool clear_viewport = true);

  void clear() const;

  void clear(const Color4& color) const;

  void clearWithoutLock() const;

  void clearWithoutLock(const Color4& color) const;

  QMatrix4x4 getPvmMatrixFromViewpoint(const Camera &camera, const Pose& pose) const;

  QMatrix4x4 getPvmMatrixFromPose(const Pose &pose) const;

  QMatrix4x4 getVmMatrixFromPose(const Pose &pose) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  friend class DrawingHandle;

  mutable std::mutex opengl_mutex_;

  mutable QOpenGLContext *opengl_context_;
  mutable QOffscreenSurface *opengl_surface_;
  mutable QOpenGLFramebufferObject *opengl_fbo_;

  bool antialiasing_;
  Color4 clear_color_;
  Camera camera_;
  FloatT near_plane_;
  FloatT far_plane_;
};

}
}

#include "offscreen_opengl.hxx"
