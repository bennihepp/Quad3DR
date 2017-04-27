//==================================================
// offscreen_opengl.hxx
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 19.04.17
//==================================================

namespace bh {
namespace opengl {

template <typename FloatT>
OffscreenOpenGL<FloatT>::DrawingHandle::DrawingHandle(OffscreenOpenGL& opengl, const bool clear_viewport)
    : finished_(false),
      opengl_(opengl) {
        lock_ = opengl_.acquireOpenGLLock();
        opengl_.bindOpenGLFbo();
        opengl_.beginOpenGLDrawing();

        if (clear_viewport) {
          opengl_.clearWithoutLock();
        }
}

template <typename FloatT>
OffscreenOpenGL<FloatT>::DrawingHandle::~DrawingHandle() {
  if (!finished_) {
    finish();
  }
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::DrawingHandle::finish() {
  opengl_.finishOpenGLDrawing();
  opengl_.releaseOpenGLFbo();
  finished_ = true;
}
template <typename FloatT>
QImage OffscreenOpenGL<FloatT>::DrawingHandle::getImageQt() {
  opengl_.opengl_context_->functions()->glFlush();

  QImage fbo_image_raw = opengl_.opengl_fbo_->toImage();
  // fbo_image_raw is Format_ARGB32_Premultiplied but the OpenGL drawing itself is not premultiplied.
  // Here we just get another view on the data with Format_ARGB32
  QImage fbo_image(fbo_image_raw.constBits(), fbo_image_raw.width(), fbo_image_raw.height(), QImage::Format_ARGB32);
  QImage image(fbo_image.width(), fbo_image.height(), QImage::Format_ARGB32);
  QPainter painter(&image);
  painter.drawImage(QPoint(0, 0), fbo_image);
  painter.end();
  //  std::memcpy(image.bits(), fbo_image.constBits(), fbo_image.byteCount());

  return image;
}

template <typename FloatT>
const OffscreenOpenGL<FloatT>* OffscreenOpenGL<FloatT>::DrawingHandle::operator->() const {
  return &opengl_;
}

template <typename FloatT>
OffscreenOpenGL<FloatT>* OffscreenOpenGL<FloatT>::DrawingHandle::operator->() {
  return &opengl_;
}

template <typename FloatT>
OffscreenOpenGL<FloatT>::OffscreenOpenGL(const Camera& camera)
        : opengl_context_(nullptr),
          opengl_surface_(nullptr),
          opengl_fbo_(nullptr),
          antialiasing_(false),
          clear_color_(1, 1, 1, 1),
          camera_(camera),
          near_plane_(0.5),
          far_plane_(1e5) {}

template <typename FloatT>
OffscreenOpenGL<FloatT>::~OffscreenOpenGL() {
  clearOpenGL();
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::setClearColor(const Color4& color) {
  clear_color_ = color;
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::setAntialiasing(const bool antialiasing) {
  if (isInitialized()) {
    const bool framebuffer_update_required = antialiasing != antialiasing_;
    if (framebuffer_update_required) {
      clearOpenGL();
    }
  }
  antialiasing_ = antialiasing;
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::setCamera(const Camera& camera) {
  camera_ = camera;
  if (isInitialized()) {
    const bool framebuffer_update_required = camera_.width() != (size_t)opengl_fbo_->width()
                                             || camera_.height() != (size_t)opengl_fbo_->height();
    if (framebuffer_update_required) {
      clearOpenGL();
    }
  }
}

template <typename FloatT>
FloatT OffscreenOpenGL<FloatT>::getNearPlane() const {
  return near_plane_;
}

template <typename FloatT>
FloatT OffscreenOpenGL<FloatT>::getFarPlane() const {
  return far_plane_;
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::setNearFarPlane(const FloatT near_plane, const FloatT far_plane) {
  near_plane_ = near_plane;
  far_plane_ = far_plane;
}

template <typename FloatT>
std::unique_lock<std::mutex> OffscreenOpenGL<FloatT>::acquireOpenGLLock() const {
  return std::unique_lock<std::mutex>(opengl_mutex_);
}

template <typename FloatT>
bool OffscreenOpenGL<FloatT>::isInitialized() const {
  return opengl_context_ != nullptr;
}

template <typename FloatT>
const QOpenGLContext* OffscreenOpenGL<FloatT>::getContext() const {
  return opengl_context_;
}

template <typename FloatT>
QOpenGLContext* OffscreenOpenGL<FloatT>::getContext() {
  return opengl_context_;
}

template <typename FloatT>
const QOffscreenSurface* OffscreenOpenGL<FloatT>::getSurface() const {
  return opengl_surface_;
}

template <typename FloatT>
QOffscreenSurface* OffscreenOpenGL<FloatT>::getSurface() {
  return opengl_surface_;
}

template <typename FloatT>
const QOpenGLFramebufferObject* OffscreenOpenGL<FloatT>::getFramebuffer() const {
  return opengl_fbo_;
}

template <typename FloatT>
QOpenGLFramebufferObject* OffscreenOpenGL<FloatT>::getFramebuffer() {
  return opengl_fbo_;
}

template <typename FloatT>
const QThread* OffscreenOpenGL<FloatT>::getContextThread() const {
  return opengl_context_->thread();
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::initializeOpenGL() const {
  opengl_context_ = new QOpenGLContext();
  QSurfaceFormat format;
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  if (antialiasing_) {
    format.setSamples(4);
  }
  else {
    format.setSamples(0);
  }
  format.setDepthBufferSize(24);
  opengl_context_->setFormat(format);
  opengl_context_->create();

  opengl_surface_ = new QOffscreenSurface();
  opengl_surface_->setFormat(format);
  opengl_surface_->create();

  opengl_context_->makeCurrent(opengl_surface_);
  QOpenGLFramebufferObjectFormat fbo_format;
  if (antialiasing_) {
    fbo_format.setSamples(4);
  }
  else {
    fbo_format.setSamples(0);
  }
  fbo_format.setAttachment(QOpenGLFramebufferObject::Depth);
  opengl_fbo_ = new QOpenGLFramebufferObject(camera_.width(), camera_.height(), fbo_format);
  opengl_fbo_->bind();
  opengl_context_->functions()->glViewport(0, 0, camera_.width(), camera_.height());
  opengl_fbo_->bindDefault();
  opengl_context_->doneCurrent();
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::clearOpenGL() const {
  if (opengl_context_ != nullptr) {
    opengl_fbo_->release();
    opengl_context_->doneCurrent();
    SAFE_DELETE(opengl_fbo_);
    SAFE_DELETE(opengl_surface_);
    SAFE_DELETE(opengl_context_);
  }
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::bindOpenGLFbo() const {
  QThread* current_thread = QThread::currentThread();
  if (opengl_context_ != nullptr && current_thread != opengl_context_->thread()) {
    std::cout << "Warning: Clearing offscreen OpenGL objects" << std::endl;
    clearOpenGL();
  }
  if (opengl_context_ == nullptr) {
    initializeOpenGL();
  }
  opengl_context_->moveToThread(current_thread);
  opengl_context_->makeCurrent(opengl_surface_);
  opengl_fbo_->bind();
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::releaseOpenGLFbo() const {
  opengl_fbo_->bindDefault();
  opengl_context_->doneCurrent();
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::beginOpenGLDrawing() const {
  if (antialiasing_) {
    opengl_context_->functions()->glEnable(GL_MULTISAMPLE);
  }
  else {
    opengl_context_->functions()->glDisable(GL_MULTISAMPLE);
  }
  opengl_context_->functions()->glEnable(GL_DEPTH_TEST);
//  glDisable(GL_DEPTH_TEST);
  opengl_context_->functions()->glEnable(GL_BLEND);
  opengl_context_->functions()->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  opengl_context_->functions()->glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

  opengl_context_->functions()->glEnable(GL_LIGHTING);
  opengl_context_->functions()->glDisable(GL_CULL_FACE);

  opengl_context_->functions()->glClearDepthf(1.0f);
  opengl_context_->functions()->glClear(GL_DEPTH_BUFFER_BIT);
  opengl_context_->functions()->glClearColor(1, 1, 1, 1);
  opengl_context_->functions()->glClear(GL_COLOR_BUFFER_BIT);
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::finishOpenGLDrawing() const {
}


template <typename FloatT>
auto OffscreenOpenGL<FloatT>::beginDrawing(const bool clear_viewport) -> DrawingHandle {
  return DrawingHandle(*this, clear_viewport);
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::clear() const {
  clear(clear_color_);
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::clear(const Color4& color) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

  clearWithoutLock(color);

  finishOpenGLDrawing();
  releaseOpenGLFbo();
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::clearWithoutLock() const {
  clearWithoutLock(clear_color_);
}

template <typename FloatT>
void OffscreenOpenGL<FloatT>::clearWithoutLock(const Color4& color) const {
  opengl_context_->functions()->glClearColor(color.r(), color.g(), color.b(), color.a());
  opengl_context_->functions()->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

template <typename FloatT>
QMatrix4x4 OffscreenOpenGL<FloatT>::getPvmMatrixFromViewpoint(const Camera& camera, const Pose& pose) const {
//  const double fy = camera.focalLengthY();
//  const double v_fov = 2 * std::atan(camera.height() / (2 * fy));
//  const qreal v_fov_degree = v_fov * 180 / (qreal)M_PI;
//  const qreal aspect_ratio = camera.width() / static_cast<qreal>(camera.height());
////  std::cout << "Setting camera FOV to " << v_fov_degree << " degrees" << std::endl;
//  QMatrix4x4 proj;
//  proj.perspective(v_fov_degree, aspect_ratio, near_plane_, far_plane_);

  QMatrix4x4 proj;
  proj.fill(0);
  proj(0, 0) = camera.intrinsics()(0, 0) / camera.intrinsics()(0, 2);
  proj(1, 1) = camera.intrinsics()(1, 1) / camera.intrinsics()(1, 2);
  proj(2, 2) = -(far_plane_ + near_plane_) / (far_plane_ - near_plane_);
  proj(2, 3) = -2 * far_plane_ * near_plane_ / (far_plane_ - near_plane_);
  proj(3, 2) = -1;

  QMatrix4x4 pvm_matrix = proj * getVmMatrixFromPose(pose);
  return pvm_matrix;
}

template <typename FloatT>
QMatrix4x4 OffscreenOpenGL<FloatT>::getPvmMatrixFromPose(const Pose& pose) const {
  return getPvmMatrixFromViewpoint(camera_, pose);
}

template <typename FloatT>
QMatrix4x4 OffscreenOpenGL<FloatT>::getVmMatrixFromPose(const Pose& pose) const {
  QMatrix4x4 model;

//  const Pose view_pose = pose.inverse();
  Pose view_pose = pose;
  // Convert to OpenGL camera coordinate system (x is right, y is up, z is back)
  AngleAxis rotate_x_pi = AngleAxis(M_PI, Vector3::UnitX());
  view_pose.quaternion() = view_pose.quaternion() * rotate_x_pi;
  Matrix3x4 world_to_image = view_pose.getTransformationWorldToImage();
  QMatrix4x4 view;
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 4; ++j) {
      view(i, j) = world_to_image(i, j);
    }
  }

  QMatrix4x4 vm_matrix = view * model;
  return vm_matrix;
}

}
}
