//==================================================
// viewpoint_offscreen_renderer.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 28.03.17
//

#include "viewpoint_offscreen_renderer.h"

namespace viewpoint_planner {

ViewpointOffscreenRenderer::ViewpointOffscreenRenderer(const PinholeCamera& camera, const MeshType* poisson_mesh)
    : opengl_context_(nullptr),
      opengl_surface_(nullptr),
      opengl_fbo_(nullptr),
      poisson_mesh_drawer_(nullptr),
      antialiasing_(false),
      clear_color_(1, 1, 1, 1),
      camera_(camera), poisson_mesh_(poisson_mesh) {}

ViewpointOffscreenRenderer::ViewpointOffscreenRenderer(
        const Options& options, const PinholeCamera& camera, const MeshType* poisson_mesh)
    : options_(options),
      opengl_context_(nullptr),
      opengl_surface_(nullptr),
      opengl_fbo_(nullptr),
      poisson_mesh_drawer_(nullptr),
      antialiasing_(false),
      clear_color_(1, 1, 1, 1),
      camera_(camera), poisson_mesh_(poisson_mesh) {}

ViewpointOffscreenRenderer::~ViewpointOffscreenRenderer() {
  clearOpenGL();
}

void ViewpointOffscreenRenderer::setClearColor(const bh::Color4<FloatType>& color) {
  clear_color_ = color;
}

void ViewpointOffscreenRenderer::setAntialiasing(const bool antialiasing) {
  if (isInitialized()) {
    const bool framebuffer_update_required = antialiasing != antialiasing_;
    if (framebuffer_update_required) {
      clearOpenGL();
    }
  }
  antialiasing_ = antialiasing;
}

void ViewpointOffscreenRenderer::setCamera(const PinholeCamera& camera) {
  camera_ = camera;
  if (isInitialized()) {
    const bool framebuffer_update_required = camera_.width() != (size_t)opengl_fbo_->width()
                                             || camera_.height() != (size_t)opengl_fbo_->height();
    if (framebuffer_update_required) {
      clearOpenGL();
    }
  }
}

std::unique_lock<std::mutex> ViewpointOffscreenRenderer::acquireOpenGLLock() const {
  return std::unique_lock<std::mutex>(opengl_mutex_);
}

bool ViewpointOffscreenRenderer::isInitialized() const {
  return opengl_context_ != nullptr;
}

void ViewpointOffscreenRenderer::initializeOpenGL() const {
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
  poisson_mesh_drawer_ = new rendering::TriangleDrawer();
  poisson_mesh_drawer_->init();
  poisson_mesh_drawer_->setDrawTriangles(true);
  uploadPoissonMesh();
  opengl_fbo_->bindDefault();
  opengl_context_->doneCurrent();
}

void ViewpointOffscreenRenderer::clearOpenGL() const {
  if (opengl_context_ != nullptr) {
    opengl_fbo_->release();
    opengl_context_->doneCurrent();
    SAFE_DELETE(opengl_fbo_);
    SAFE_DELETE(opengl_surface_);
    SAFE_DELETE(opengl_context_);
  }
}

void ViewpointOffscreenRenderer::uploadPoissonMesh() const {
//  std::cout << "Uploading poisson_mesh" << std::endl;
  std::vector<rendering::OGLTriangleWithNormalData> triangle_normal_data;
  triangle_normal_data.reserve(poisson_mesh_->m_FaceIndicesVertices.size());
  const ml::BoundingBox3<FloatType> mesh_bbox = poisson_mesh_->computeBoundingBox();
  const bh::ColorMapJet<FloatType> cmap;
  for (size_t i = 0; i < poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const ViewpointOffscreenRenderer::MeshType::Indices::Face& vertex_indices = poisson_mesh_->m_FaceIndicesVertices[i];
    BH_ASSERT_STR(vertex_indices.size() == 3, "Mesh face vertex indices need to have a valence of 3");
    const ml::vec3f& v1 = poisson_mesh_->m_Vertices[vertex_indices[0]];
    const ml::vec3f& v2 = poisson_mesh_->m_Vertices[vertex_indices[1]];
    const ml::vec3f& v3 = poisson_mesh_->m_Vertices[vertex_indices[2]];
    ml::vec3f n1;
    ml::vec3f n2;
    ml::vec3f n3;
    if (poisson_mesh_->hasNormalIndices() && poisson_mesh_->hasNormals()) {
      const ViewpointOffscreenRenderer::MeshType::Indices::Face& normal_indices = poisson_mesh_->m_FaceIndicesNormals[i];
      BH_ASSERT_STR(normal_indices.size() == 3, "Mesh face normal indices need to have a valence of 3");
      n1 = ml::vec3f::normalize(poisson_mesh_->m_Normals[normal_indices[0]]);
      n2 = ml::vec3f::normalize(poisson_mesh_->m_Normals[normal_indices[1]]);
      n3 = ml::vec3f::normalize(poisson_mesh_->m_Normals[normal_indices[2]]);
    } else {
      n1 = ml::vec3f::normalize(ml::vec3f::cross(v2 - v1, v3 - v2));
      n2 = n1;
      n3 = n1;
    }
    ml::vec4f c1;
    ml::vec4f c2;
    ml::vec4f c3;
    if (poisson_mesh_->hasColors()) {
      c1 = poisson_mesh_->m_Colors[vertex_indices[0]];
      c2 = poisson_mesh_->m_Colors[vertex_indices[1]];
      c3 = poisson_mesh_->m_Colors[vertex_indices[2]];
    }
    else {
      const FloatType coeff1 = (v1.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const FloatType coeff2 = (v2.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const FloatType coeff3 = (v3.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const bh::Color3<FloatType> color1 = cmap.map(coeff1);
      const bh::Color3<FloatType> color2 = cmap.map(coeff2);
      const bh::Color3<FloatType> color3 = cmap.map(coeff3);
      c1 = ml::vec4f(color1.r(), color1.g(), color1.b(), 1);
      c2 = ml::vec4f(color2.r(), color2.g(), color2.b(), 1);
      c3 = ml::vec4f(color3.r(), color3.g(), color3.b(), 1);
    }
    const FloatType a1 = 1;
    const FloatType a2 = 1;
    const FloatType a3 = 1;
    rendering::OGLTriangleWithNormalData triangle_normal;
    triangle_normal.vertex1 = rendering::OGLVertexNormalDataRGBA(v1.x, v1.y, v1.z, n1.x, n1.y, n1.z, c1.r, c1.g, c1.b, a1);
    triangle_normal.vertex2 = rendering::OGLVertexNormalDataRGBA(v2.x, v2.y, v2.z, n2.x, n2.y, n2.z, c2.r, c2.g, c2.b, a2);
    triangle_normal.vertex3 = rendering::OGLVertexNormalDataRGBA(v3.x, v3.y, v3.z, n3.x, n3.y, n3.z, c3.r, c3.g, c3.b, a3);
//    triangle_normal.vertex1.r = n1.x;
//    triangle_normal.vertex1.g = n1.y;
//    triangle_normal.vertex1.b = n1.z;
//    triangle_normal.vertex2.r = n2.x;
//    triangle_normal.vertex2.g = n2.y;
//    triangle_normal.vertex2.b = n2.z;
//    triangle_normal.vertex3.r = n3.x;
//    triangle_normal.vertex3.g = n3.y;
//    triangle_normal.vertex3.b = n3.z;
    triangle_normal_data.push_back(triangle_normal);
  }
  std::cout << "Uploading " << triangle_normal_data.size() << " triangles" << std::endl;
  poisson_mesh_drawer_->upload(triangle_normal_data);
}

void ViewpointOffscreenRenderer::bindOpenGLFbo() const {
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

void ViewpointOffscreenRenderer::releaseOpenGLFbo() const {
  opengl_fbo_->bindDefault();
  opengl_context_->doneCurrent();
}

void ViewpointOffscreenRenderer::beginOpenGLDrawing() const {
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

void ViewpointOffscreenRenderer::finishOpenGLDrawing() const {
}

void ViewpointOffscreenRenderer::clear() const {
  clear(clear_color_);
}

void ViewpointOffscreenRenderer::clear(const bh::Color4<FloatType>& color) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

  opengl_context_->functions()->glClearColor(color.r(), color.g(), color.b(), color.a());
  opengl_context_->functions()->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  finishOpenGLDrawing();
  releaseOpenGLFbo();
}

void ViewpointOffscreenRenderer::clearWithoutLock() const {
  clearWithoutLock(clear_color_);
}

void ViewpointOffscreenRenderer::clearWithoutLock(const bh::Color4<FloatType>& color) const {
  opengl_context_->functions()->glClearColor(color.r(), color.g(), color.b(), color.a());
  opengl_context_->functions()->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
}

QMatrix4x4 ViewpointOffscreenRenderer::getPvmMatrixFromViewpoint(const Viewpoint& viewpoint) const {
  const double fy = viewpoint.camera().getFocalLengthY();
  const double v_fov = 2 * std::atan(viewpoint.camera().height() / (2 * fy));
  const qreal v_fov_degree = v_fov * 180 / (qreal)M_PI;
  const qreal aspect_ratio = viewpoint.camera().width() / static_cast<qreal>(viewpoint.camera().height());
  const qreal near_plane = 0.5;
  const qreal far_plane = 1e5;
//  std::cout << "Setting camera FOV to " << v_fov_degree << " degrees" << std::endl;
  QMatrix4x4 proj;
  proj.perspective(v_fov_degree, aspect_ratio, near_plane, far_plane);

  QMatrix4x4 pvm_matrix = proj * getVmMatrixFromViewpoint(viewpoint);
  return pvm_matrix;
}

QMatrix4x4 ViewpointOffscreenRenderer::getPvmMatrixFromPose(const Pose& pose) const {
  return getPvmMatrixFromViewpoint(Viewpoint(&camera_, pose));
}

QMatrix4x4 ViewpointOffscreenRenderer::getVmMatrixFromViewpoint(const Viewpoint& viewpoint) const {
  return getVmMatrixFromPose(viewpoint.pose());
}

QMatrix4x4 ViewpointOffscreenRenderer::getVmMatrixFromPose(const Pose& pose) const {
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

QImage ViewpointOffscreenRenderer::drawPoissonMesh(const Viewpoint& viewpoint, const bool clear_viewport) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromViewpoint(viewpoint);
  return drawPoissonMesh(pvm_matrix, clear_viewport);
}

QImage ViewpointOffscreenRenderer::drawPoissonMesh(const Pose& pose, const bool clear_viewport) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromPose(pose);
  return drawPoissonMesh(pvm_matrix, clear_viewport);
}

QImage ViewpointOffscreenRenderer::drawPoissonMesh(const QMatrix4x4& pvm_matrix, const bool clear_viewport) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

  if (clear_viewport) {
    clearWithoutLock();
  }

  poisson_mesh_drawer_->draw(pvm_matrix, opengl_context_->functions());
  opengl_context_->functions()->glFlush();

  QImage fbo_image_raw = opengl_fbo_->toImage();
  // fbo_image_raw is Format_ARGB32_Premultiplied but the OpenGL drawing itself is not premultiplied.
  // Here we just get another view on the data with Format_ARGB32
  QImage fbo_image(fbo_image_raw.constBits(), fbo_image_raw.width(), fbo_image_raw.height(), QImage::Format_ARGB32);
  QImage image(fbo_image.width(), fbo_image.height(), QImage::Format_ARGB32);
  QPainter painter(&image);
  painter.drawImage(QPoint(0, 0), fbo_image);
  painter.end();
//  std::memcpy(image.bits(), fbo_image.constBits(), fbo_image.byteCount());

  finishOpenGLDrawing();
  releaseOpenGLFbo();

  return image;
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshNormals(const Viewpoint& viewpoint, const bool clear_viewport) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromViewpoint(viewpoint);
  return drawPoissonMeshNormals(pvm_matrix, clear_viewport);
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshNormals(const Pose& pose, const bool clear_viewport) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromPose(pose);
  return drawPoissonMeshNormals(pvm_matrix, clear_viewport);
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshNormals(const QMatrix4x4& pvm_matrix, const bool clear_viewport) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

  if (clear_viewport) {
    clearWithoutLock();
  }

  poisson_mesh_drawer_->drawNormals(pvm_matrix, opengl_context_->functions());
  opengl_context_->functions()->glFlush();

  QImage fbo_image_raw = opengl_fbo_->toImage();
  // fbo_image_raw is Format_ARGB32_Premultiplied but the OpenGL drawing itself is not premultiplied.
  // Here we just get another view on the data with Format_ARGB32
  QImage fbo_image(fbo_image_raw.constBits(), fbo_image_raw.width(), fbo_image_raw.height(), QImage::Format_ARGB32);
  QImage image(fbo_image.width(), fbo_image.height(), QImage::Format_ARGB32);
  QPainter painter(&image);
  painter.drawImage(QPoint(0, 0), fbo_image);
  painter.end();
//  std::memcpy(image.bits(), fbo_image.constBits(), fbo_image.byteCount());

  finishOpenGLDrawing();
  releaseOpenGLFbo();

  return image;
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshDepth(const Viewpoint& viewpoint, const bool clear_viewport) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromViewpoint(viewpoint);
  const QMatrix4x4 vm_matrix = getVmMatrixFromViewpoint(viewpoint);
  return drawPoissonMeshDepth(pvm_matrix, vm_matrix, clear_viewport);
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshDepth(const Pose& pose, const bool clear_viewport) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromPose(pose);
  const QMatrix4x4 vm_matrix = getVmMatrixFromPose(pose);
  return drawPoissonMeshDepth(pvm_matrix, vm_matrix, clear_viewport);
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshDepth(
        const QMatrix4x4& pvm_matrix, const QMatrix4x4& vm_matrix, const bool clear_viewport) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

  if (clear_viewport) {
    clearWithoutLock();
  }

  poisson_mesh_drawer_->drawDepth(pvm_matrix, vm_matrix, opengl_context_->functions());
  opengl_context_->functions()->glFlush();

  QImage fbo_image_raw = opengl_fbo_->toImage();
  // fbo_image_raw is Format_ARGB32_Premultiplied but the OpenGL drawing itself is not premultiplied.
  // Here we just get another view on the data with Format_ARGB32
  QImage fbo_image(fbo_image_raw.constBits(), fbo_image_raw.width(), fbo_image_raw.height(), QImage::Format_ARGB32);
  QImage image(fbo_image.width(), fbo_image.height(), QImage::Format_ARGB32);
  QPainter painter(&image);
  painter.drawImage(QPoint(0, 0), fbo_image);
  painter.end();
//  std::memcpy(image.bits(), fbo_image.constBits(), fbo_image.byteCount());

//  Eigen::MatrixXf float_depth_image(opengl_fbo_->height(), opengl_fbo_->width());
//  float_depth_image.setZero();
//  opengl_context_->functions()->glReadPixels(
//      0, 0, opengl_fbo_->width(), opengl_fbo_->height(),
//      GL_DEPTH_COMPONENT, GL_FLOAT, float_depth_image.data());
//  BH_PRINT_VALUE(float_depth_image.minCoeff());
//  BH_PRINT_VALUE(float_depth_image.maxCoeff());

  finishOpenGLDrawing();
  releaseOpenGLFbo();

  return fbo_image_raw;
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshIndices(const Viewpoint& viewpoint, const bool clear_viewport) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromViewpoint(viewpoint);
  return drawPoissonMeshIndices(pvm_matrix, clear_viewport);
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshIndices(const Pose& pose, const bool clear_viewport) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromPose(pose);
  return drawPoissonMeshIndices(pvm_matrix, clear_viewport);
}

QImage ViewpointOffscreenRenderer::drawPoissonMeshIndices(const QMatrix4x4& pvm_matrix, const bool clear_viewport) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

  if (clear_viewport) {
    clearWithoutLock();
  }

  poisson_mesh_drawer_->drawIndices(pvm_matrix, opengl_context_->functions());
  opengl_context_->functions()->glFlush();

  QImage fbo_image_raw = opengl_fbo_->toImage();
  // fbo_image_raw is Format_ARGB32_Premultiplied but the OpenGL drawing itself is not premultiplied.
  // Here we just get another view on the data with Format_ARGB32
  QImage fbo_image(fbo_image_raw.constBits(), fbo_image_raw.width(), fbo_image_raw.height(), QImage::Format_ARGB32);
  QImage image(fbo_image.width(), fbo_image.height(), QImage::Format_ARGB32);
  QPainter painter(&image);
  painter.drawImage(QPoint(0, 0), fbo_image);
  painter.end();
//  std::memcpy(image.bits(), fbo_image.constBits(), fbo_image.byteCount());

  finishOpenGLDrawing();
  releaseOpenGLFbo();

  return image;
}

Vector3 ViewpointOffscreenRenderer::computePoissonMeshNormalVector(
        const Viewpoint& viewpoint, const Vector3& position) const {
  using Vector2i = typename bh::EigenTypes<int>::Vector2;
  const Vector2i point2d = viewpoint.projectWorldPointIntoImage(position).cast<int>();
#if !BH_RELEASE
  BH_ASSERT(point2d(0) >= 0 && point2d(0) < (int)viewpoint.camera().width());
  BH_ASSERT(point2d(1) >= 0 && point2d(1) < (int)viewpoint.camera().width());
#endif
//  const std::size_t x = static_cast<std::size_t>(std::floor(point2d(0)));
//  const std::size_t y = static_cast<std::size_t>(std::floor(point2d(1)));
  return computePoissonMeshNormalVector(viewpoint, point2d(0), point2d(1));
}

Vector3 ViewpointOffscreenRenderer::computePoissonMeshNormalVector(
        const Viewpoint& viewpoint,
        const std::size_t x, const std::size_t y) const {
  const Pose& pose = viewpoint.pose();
  std::unique_lock<std::mutex> cache_lock(poisson_mesh_cache_mutex_);
  if (!pose.isApprox(cached_poisson_mesh_normals_pose_) || cached_poisson_mesh_normals_image_.width() == 0) {
//    std::cout << "Recomputing poisson mesh normals" << std::endl;
    cached_poisson_mesh_normals_pose_ = pose;
    cached_poisson_mesh_normals_image_ = drawPoissonMeshNormals(pose);
//    BH_PRINT_VALUE(cached_poisson_mesh_normals_image_.width());
//    BH_PRINT_VALUE(cached_poisson_mesh_normals_image_.height());
    if (options_.dump_poisson_mesh_normals_image) {
      cached_poisson_mesh_normals_image_.save("dump_poisson_mesh_normals_image.png");
    }
  }
  cache_lock.unlock();
#if !BH_RELEASE
  BH_ASSERT(x >= 0 && x < (std::size_t)cached_poisson_mesh_normals_image_.width());
  BH_ASSERT(y >= 0 && y < (std::size_t)cached_poisson_mesh_normals_image_.height());
#endif
  QColor pixel = QColor(cached_poisson_mesh_normals_image_.pixel(x, y));
  Vector3 normal_vector = Vector3(pixel.red(), pixel.green(), pixel.blue()) / 255.f;
  normal_vector = 2 * (normal_vector - Vector3(0.5f, 0.5f, 0.5f));
  if (normal_vector.squaredNorm() < 0.5f) {
    normal_vector = Vector3::Zero();
  }
  normal_vector.normalize();
  return normal_vector;
}

FloatType ViewpointOffscreenRenderer::computePoissonMeshDepth(
        const Viewpoint& viewpoint, const Vector3& position) const {
  const Vector2 point2d = viewpoint.projectWorldPointIntoImage(position);
#if !BH_RELEASE
  BH_ASSERT(point2d(0) >= 0 && point2d(0) < viewpoint.camera().width());
  BH_ASSERT(point2d(1) >= 0 && point2d(1) < viewpoint.camera().width());
#endif
  const std::size_t x = static_cast<std::size_t>(std::round(point2d(0)));
  const std::size_t y = static_cast<std::size_t>(std::round(point2d(1)));
  return computePoissonMeshDepth(viewpoint, x, y);
}

FloatType ViewpointOffscreenRenderer::computePoissonMeshDepth(
        const Viewpoint& viewpoint,
        const std::size_t x, const std::size_t y) const {
  const Pose& pose = viewpoint.pose();
  std::unique_lock<std::mutex> cache_lock(poisson_mesh_cache_mutex_);
  if (!pose.isApprox(cached_poisson_mesh_depth_pose_) || cached_poisson_mesh_depth_image_.width() == 0) {
//    std::cout << "Recomputing poisson mesh depth" << std::endl;
    cached_poisson_mesh_depth_pose_ = pose;
    cached_poisson_mesh_depth_image_ = drawPoissonMeshDepth(pose);
    if (options_.dump_poisson_mesh_depth_image) {
      cached_poisson_mesh_depth_image_.save("dump_poisson_mesh_depth_image.png");
      QImage rgb_depth_image = convertEncodedDepthImageToRGB(cached_poisson_mesh_depth_image_, 0, 100);
      rgb_depth_image.save("dump_poisson_mesh_depth_rgb_image.png");
    }
#if !BH_RELEASE
    BH_ASSERT(cached_poisson_mesh_depth_image_.width() != 0);
    BH_ASSERT(cached_poisson_mesh_depth_image_.height() != 0);
#endif
  }
  cache_lock.unlock();
#if !BH_RELEASE
  BH_ASSERT(x >= 0 && x < (std::size_t)cached_poisson_mesh_depth_image_.width());
  BH_ASSERT(y >= 0 && y < (std::size_t)cached_poisson_mesh_depth_image_.height());
#endif
  QColor pixel = QColor(cached_poisson_mesh_depth_image_.pixel(x, y));
  return decodeDepthValue(pixel);
}

bh::Color4<uint8_t> ViewpointOffscreenRenderer::encodeDepthValue(const FloatType depth) const {
  const FloatType multiplier(256.);
  const FloatType red = std::floor(depth / multiplier);
  const FloatType green = std::floor(depth - red * multiplier);
  const FloatType blue = std::floor((depth - red * multiplier - green) * multiplier);
  bh::Color4<uint8_t> color(
          static_cast<uint8_t>(bh::clamp<FloatType>(red, 0, 255)),
          static_cast<uint8_t>(bh::clamp<FloatType>(green, 0, 255)),
          static_cast<uint8_t>(bh::clamp<FloatType>(blue, 0, 255)), 255);
  return color;
}

FloatType ViewpointOffscreenRenderer::decodeDepthValue(
        const QImage& depth_image, const Vector2& image_point) const {
  return decodeDepthValue(QColor(depth_image.pixel(image_point(0), image_point(1))));
}

FloatType ViewpointOffscreenRenderer::decodeDepthValue(const bh::Color4<uint8_t>& color) const {
  const FloatType depth = color.r() * FloatType(256) + color.g() + color.b() / FloatType(256.);
  return depth;
}

FloatType ViewpointOffscreenRenderer::decodeDepthValue(const QColor& pixel) const {
  const FloatType depth = pixel.red() * FloatType(256) + pixel.green() + pixel.blue() / FloatType(256.);
  return depth;
}

QImage ViewpointOffscreenRenderer::convertEncodedDepthImageToRGB(
        const QImage& encoded_image, const FloatType min_depth, const FloatType max_depth) const {
  Eigen::MatrixXf depth_matrix(encoded_image.height(), encoded_image.width());
  for (int y = 0; y < encoded_image.height(); ++y) {
    for (int x = 0; x < encoded_image.width(); ++x) {
      depth_matrix(y, x) = decodeDepthValue(QColor(encoded_image.pixel(x, y)));
    }
  }
  QImage rgb_image(encoded_image.width(), encoded_image.height(), QImage::Format_ARGB32);
  bh::ColorMapHot<FloatType> cmap;
  for (int y = 0; y < encoded_image.height(); ++y) {
    for (int x = 0; x < encoded_image.width(); ++x) {
      const FloatType depth = bh::clamp(depth_matrix(y, x), min_depth, max_depth);
      const bh::Color3<FloatType> color = cmap.map(bh::normalize(depth, min_depth, max_depth));
      const QColor pixel(
              bh::clamp<int>(255 * color.r(), 0, 255),
              bh::clamp<int>(255 * color.g(), 0, 255),
              bh::clamp<int>(255 * color.b(), 0, 255));
      rgb_image.setPixel(x, y, pixel.rgba());
    }
  }
  return rgb_image;
}

QImage ViewpointOffscreenRenderer::convertEncodedIndicesImageToRGB(
        const QImage& encoded_image, const size_t min_index, const size_t max_index) const {
  typename bh::EigenTypes<size_t>::MatrixDynamic indices_matrix(encoded_image.height(), encoded_image.width());
  for (int y = 0; y < encoded_image.height(); ++y) {
    for (int x = 0; x < encoded_image.width(); ++x) {
      const QColor color(encoded_image.pixel(x, y));
      const size_t index = poisson_mesh_drawer_->colorToIndex(color);
      indices_matrix(y, x) = index;
    }
  }
  QImage rgb_image(encoded_image.width(), encoded_image.height(), QImage::Format_ARGB32);
  bh::ColorMapHot<FloatType> cmap;
  for (int y = 0; y < encoded_image.height(); ++y) {
    for (int x = 0; x < encoded_image.width(); ++x) {
      const size_t index = indices_matrix(y, x);
      const size_t clamped_index = bh::clamp(index, min_index, max_index);
      const bh::Color3<FloatType> color = cmap.map(bh::normalize<FloatType>(clamped_index, min_index, max_index));
      const QColor pixel(
              bh::clamp<int>(255 * color.r(), 0, 255),
              bh::clamp<int>(255 * color.g(), 0, 255),
              bh::clamp<int>(255 * color.b(), 0, 255));
      rgb_image.setPixel(x, y, pixel.rgba());
    }
  }
  return rgb_image;
}

std::unordered_set<size_t> ViewpointOffscreenRenderer::getVisibleTriangles(const QImage& mesh_indices_image) const {
  const QColor invalid_color = QColor::fromRgb(255, 255, 255, 255);
  const size_t invalid_index = poisson_mesh_drawer_->colorToIndex(invalid_color);
  std::unordered_set<size_t> visible_triangles;
  for (int y = 0; y < mesh_indices_image.height(); ++y) {
    for (int x = 0; x < mesh_indices_image.width(); ++x) {
      const QColor color(mesh_indices_image.pixel(x, y));
      const size_t index = poisson_mesh_drawer_->colorToIndex(color);
      if (index != invalid_index) {
        BH_ASSERT(index < poisson_mesh_->m_FaceIndicesVertices.size());
        visible_triangles.emplace(index);
      }
    }
  }
  return visible_triangles;
}

}