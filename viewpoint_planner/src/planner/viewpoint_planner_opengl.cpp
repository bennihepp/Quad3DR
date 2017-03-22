//==================================================
// viewpoint_planner_opengl.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 5, 2017
//==================================================

#include <ait/color.h>
#include "viewpoint_planner.h"


#if WITH_OPENGL_OFFSCREEN

std::unique_lock<std::mutex> ViewpointPlanner::acquireOpenGLLock() const {
  return std::unique_lock<std::mutex>(opengl_mutex_);
}

void ViewpointPlanner::initializeOpenGL() const {
  opengl_context_ = new QOpenGLContext();
  QSurfaceFormat format;
  format.setVersion(3, 3);
  format.setProfile(QSurfaceFormat::CoreProfile);
  format.setSamples(4);
  format.setDepthBufferSize(24);
  opengl_context_->setFormat(format);
  opengl_context_->create();

  opengl_surface_ = new QOffscreenSurface();
  opengl_surface_->setFormat(format);
  opengl_surface_->create();

  opengl_context_->makeCurrent(opengl_surface_);
  QOpenGLFramebufferObjectFormat fbo_format;
  fbo_format.setSamples(4);
  fbo_format.setAttachment(QOpenGLFramebufferObject::Depth);
  opengl_fbo_ = new QOpenGLFramebufferObject(getVirtualCamera().width(), getVirtualCamera().height(), fbo_format);
  opengl_fbo_->bind();
  opengl_context_->functions()->glViewport(0, 0, getVirtualCamera().width(), getVirtualCamera().height());
  poisson_mesh_drawer_ = new TriangleDrawer();
  poisson_mesh_drawer_->init();
  poisson_mesh_drawer_->setDrawTriangles(true);
  uploadPoissonMesh();
  opengl_fbo_->bindDefault();
  opengl_context_->doneCurrent();
}

void ViewpointPlanner::clearOpenGL() const {
  opengl_fbo_->release();
  opengl_context_->doneCurrent();
  SAFE_DELETE(opengl_fbo_);
  SAFE_DELETE(opengl_surface_);
  SAFE_DELETE(opengl_context_);
}

void ViewpointPlanner::uploadPoissonMesh() const {
//  std::cout << "Uploading poisson_mesh" << std::endl;
  std::vector<OGLTriangleWithNormalData> triangle_normal_data;
  triangle_normal_data.reserve(data_->poisson_mesh_->m_FaceIndicesVertices.size());
  const ml::BoundingBox3<FloatType> mesh_bbox = data_->poisson_mesh_->computeBoundingBox();
  const ait::ColorMapJet<FloatType> cmap;
  for (size_t i = 0; i < data_->poisson_mesh_->m_FaceIndicesVertices.size(); ++i) {
    const ViewpointPlanner::MeshType::Indices::Face& vertex_indices = data_->poisson_mesh_->m_FaceIndicesVertices[i];
    AIT_ASSERT_STR(vertex_indices.size() == 3, "Mesh face vertex indices need to have a valence of 3");
    const ml::vec3f& v1 = data_->poisson_mesh_->m_Vertices[vertex_indices[0]];
    const ml::vec3f& v2 = data_->poisson_mesh_->m_Vertices[vertex_indices[1]];
    const ml::vec3f& v3 = data_->poisson_mesh_->m_Vertices[vertex_indices[2]];
    ml::vec3f n1;
    ml::vec3f n2;
    ml::vec3f n3;
    if (data_->poisson_mesh_->hasNormalIndices() && data_->poisson_mesh_->hasNormals()) {
      const ViewpointPlanner::MeshType::Indices::Face& normal_indices = data_->poisson_mesh_->m_FaceIndicesNormals[i];
      AIT_ASSERT_STR(normal_indices.size() == 3, "Mesh face normal indices need to have a valence of 3");
      n1 = ml::vec3f::normalize(data_->poisson_mesh_->m_Normals[normal_indices[0]]);
      n2 = ml::vec3f::normalize(data_->poisson_mesh_->m_Normals[normal_indices[1]]);
      n3 = ml::vec3f::normalize(data_->poisson_mesh_->m_Normals[normal_indices[2]]);
    } else {
      n1 = ml::vec3f::normalize(ml::vec3f::cross(v2 - v1, v3 - v2));
      n2 = n1;
      n3 = n1;
    }
    ml::vec4f c1;
    ml::vec4f c2;
    ml::vec4f c3;
    if (data_->poisson_mesh_->hasColors()) {
      c1 = data_->poisson_mesh_->m_Colors[vertex_indices[0]];
      c2 = data_->poisson_mesh_->m_Colors[vertex_indices[1]];
      c3 = data_->poisson_mesh_->m_Colors[vertex_indices[2]];
    }
    else {
      const FloatType coeff1 = (v1.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const FloatType coeff2 = (v2.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const FloatType coeff3 = (v3.z - mesh_bbox.getMinZ()) / mesh_bbox.getExtentZ();
      const ait::Color3<FloatType> color1 = cmap.map(coeff1);
      const ait::Color3<FloatType> color2 = cmap.map(coeff2);
      const ait::Color3<FloatType> color3 = cmap.map(coeff3);
      c1 = ml::vec4f(color1.r(), color1.g(), color1.b(), 1);
      c2 = ml::vec4f(color2.r(), color2.g(), color2.b(), 1);
      c3 = ml::vec4f(color3.r(), color3.g(), color3.b(), 1);
    }
    const FloatType a1 = 1;
    const FloatType a2 = 1;
    const FloatType a3 = 1;
    OGLTriangleWithNormalData triangle_normal;
    triangle_normal.vertex1 = OGLVertexNormalDataRGBA(v1.x, v1.y, v1.z, n1.x, n1.y, n1.z, c1.r, c1.g, c1.b, a1);
    triangle_normal.vertex2 = OGLVertexNormalDataRGBA(v2.x, v2.y, v2.z, n2.x, n2.y, n2.z, c2.r, c2.g, c2.b, a2);
    triangle_normal.vertex3 = OGLVertexNormalDataRGBA(v3.x, v3.y, v3.z, n3.x, n3.y, n3.z, c3.r, c3.g, c3.b, a3);
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

void ViewpointPlanner::bindOpenGLFbo() const {
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

void ViewpointPlanner::releaseOpenGLFbo() const {
  opengl_fbo_->bindDefault();
  opengl_context_->doneCurrent();
}

void ViewpointPlanner::beginOpenGLDrawing() const {
  opengl_context_->functions()->glEnable(GL_DEPTH_TEST);
//  glDisable(GL_DEPTH_TEST);
  opengl_context_->functions()->glEnable(GL_BLEND);
  opengl_context_->functions()->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  opengl_context_->functions()->glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

  opengl_context_->functions()->glEnable(GL_LIGHTING);
  opengl_context_->functions()->glDisable(GL_CULL_FACE);

  opengl_context_->functions()->glClear(GL_COLOR_BUFFER_BIT);
  opengl_context_->functions()->glClear(GL_DEPTH_BUFFER_BIT);
  opengl_context_->functions()->glClearColor(255, 255, 255, 1);
}

void ViewpointPlanner::finishOpenGLDrawing() const {
}

QMatrix4x4 ViewpointPlanner::getPvmMatrixFromViewpoint(const Viewpoint& viewpoint) const {
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

QMatrix4x4 ViewpointPlanner::getPvmMatrixFromPose(const Pose& pose) const {
  return getPvmMatrixFromViewpoint(getVirtualViewpoint(pose));
}

QMatrix4x4 ViewpointPlanner::getVmMatrixFromViewpoint(const Viewpoint& viewpoint) const {
  return getVmMatrixFromPose(viewpoint.pose());
}

QMatrix4x4 ViewpointPlanner::getVmMatrixFromPose(const Pose& pose) const {
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

QImage ViewpointPlanner::drawPoissonMesh(const ViewpointEntryIndex viewpoint_index) const {
  return drawPoissonMesh(viewpoint_entries_[viewpoint_index].viewpoint);
}

QImage ViewpointPlanner::drawPoissonMesh(const Viewpoint& viewpoint) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromViewpoint(viewpoint);
  return drawPoissonMesh(pvm_matrix);
}

QImage ViewpointPlanner::drawPoissonMesh(const Pose& pose) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromPose(pose);
  return drawPoissonMesh(pvm_matrix);
}

QImage ViewpointPlanner::drawPoissonMesh(const QMatrix4x4& pvm_matrix) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

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

QImage ViewpointPlanner::drawPoissonMeshNormals(const ViewpointEntryIndex viewpoint_index) const {
  return drawPoissonMeshNormals(viewpoint_entries_[viewpoint_index].viewpoint);
}

QImage ViewpointPlanner::drawPoissonMeshNormals(const Viewpoint& viewpoint) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromViewpoint(viewpoint);
  return drawPoissonMeshNormals(pvm_matrix);
}

QImage ViewpointPlanner::drawPoissonMeshNormals(const Pose& pose) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromPose(pose);
  return drawPoissonMeshNormals(pvm_matrix);
}

QImage ViewpointPlanner::drawPoissonMeshNormals(const QMatrix4x4& pvm_matrix) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

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

QImage ViewpointPlanner::drawPoissonMeshDepth(const ViewpointEntryIndex viewpoint_index) const {
  return drawPoissonMeshDepth(viewpoint_entries_[viewpoint_index].viewpoint);
}

QImage ViewpointPlanner::drawPoissonMeshDepth(const Viewpoint& viewpoint) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromViewpoint(viewpoint);
  const QMatrix4x4 vm_matrix = getVmMatrixFromViewpoint(viewpoint);
  return drawPoissonMeshDepth(pvm_matrix, vm_matrix);
}

QImage ViewpointPlanner::drawPoissonMeshDepth(const Pose& pose) const {
  const QMatrix4x4 pvm_matrix = getPvmMatrixFromPose(pose);
  const QMatrix4x4 vm_matrix = getVmMatrixFromPose(pose);
  return drawPoissonMeshDepth(pvm_matrix, vm_matrix);
}

QImage ViewpointPlanner::drawPoissonMeshDepth(const QMatrix4x4& pvm_matrix, const QMatrix4x4& vm_matrix) const {
  std::unique_lock<std::mutex> lock = acquireOpenGLLock();
  bindOpenGLFbo();
  beginOpenGLDrawing();

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
//  AIT_PRINT_VALUE(float_depth_image.minCoeff());
//  AIT_PRINT_VALUE(float_depth_image.maxCoeff());

  finishOpenGLDrawing();
  releaseOpenGLFbo();

  return fbo_image_raw;
}

ViewpointPlanner::Vector3 ViewpointPlanner::computePoissonMeshNormalVector(
    const Viewpoint& viewpoint, const Vector3& position) const {
  const Vector2 point2d = viewpoint.projectWorldPointIntoImage(position);
#if !AIT_RELEASE
  AIT_ASSERT(point2d(0) >= 0 && point2d(0) < viewpoint.camera().width());
  AIT_ASSERT(point2d(1) >= 0 && point2d(1) < viewpoint.camera().width());
#endif
  const std::size_t x = static_cast<std::size_t>(std::round(point2d(0)));
  const std::size_t y = static_cast<std::size_t>(std::round(point2d(1)));
  return computePoissonMeshNormalVector(viewpoint, x, y);
}

ViewpointPlanner::Vector3 ViewpointPlanner::computePoissonMeshNormalVector(
    const Viewpoint& viewpoint,
    const std::size_t x, const std::size_t y) const {
  const Pose& pose = viewpoint.pose();
  std::unique_lock<std::mutex> cache_lock(poisson_mesh_cache_mutex_);
  if (!pose.isApprox(cached_poisson_mesh_normals_pose_) || cached_poisson_mesh_normals_image_.width() == 0) {
//    std::cout << "Recomputing poisson mesh normals" << std::endl;
    cached_poisson_mesh_normals_pose_ = pose;
    cached_poisson_mesh_normals_image_ = drawPoissonMeshNormals(pose);
//    AIT_PRINT_VALUE(cached_poisson_mesh_normals_image_.width());
//    AIT_PRINT_VALUE(cached_poisson_mesh_normals_image_.height());
    if (options_.dump_poisson_mesh_normals_image) {
      cached_poisson_mesh_normals_image_.save("dump_poisson_mesh_normals_image.png");
    }
  }
  cache_lock.unlock();
#if !AIT_RELEASE
  AIT_ASSERT(x >= 0 && x < (std::size_t)cached_poisson_mesh_normals_image_.width());
  AIT_ASSERT(y >= 0 && y < (std::size_t)cached_poisson_mesh_normals_image_.height());
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

ViewpointPlanner::FloatType ViewpointPlanner::computePoissonMeshDepth(
    const Viewpoint& viewpoint, const Vector3& position) const {
  const Vector2 point2d = viewpoint.projectWorldPointIntoImage(position);
#if !AIT_RELEASE
  AIT_ASSERT(point2d(0) >= 0 && point2d(0) < viewpoint.camera().width());
  AIT_ASSERT(point2d(1) >= 0 && point2d(1) < viewpoint.camera().width());
#endif
  const std::size_t x = static_cast<std::size_t>(std::round(point2d(0)));
  const std::size_t y = static_cast<std::size_t>(std::round(point2d(1)));
  return computePoissonMeshDepth(viewpoint, x, y);
}

ViewpointPlanner::FloatType ViewpointPlanner::computePoissonMeshDepth(
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
#if !AIT_RELEASE
    AIT_ASSERT(cached_poisson_mesh_depth_image_.width() != 0);
    AIT_ASSERT(cached_poisson_mesh_depth_image_.height() != 0);
#endif
  }
  cache_lock.unlock();
#if !AIT_RELEASE
  AIT_ASSERT(x >= 0 && x < (std::size_t)cached_poisson_mesh_depth_image_.width());
  AIT_ASSERT(y >= 0 && y < (std::size_t)cached_poisson_mesh_depth_image_.height());
#endif
  QColor pixel = QColor(cached_poisson_mesh_depth_image_.pixel(x, y));
  return decodeDepthValue(pixel);
}

ait::Color4<uint8_t> ViewpointPlanner::encodeDepthValue(const FloatType depth) const {
  const FloatType multiplier(256.);
  const FloatType red = std::floor(depth / multiplier);
  const FloatType green = std::floor(depth - red * multiplier);
  const FloatType blue = std::floor((depth - red * multiplier - green) * multiplier);
  ait::Color4<uint8_t> color(
      static_cast<uint8_t>(ait::clamp<FloatType>(red, 0, 255)),
      static_cast<uint8_t>(ait::clamp<FloatType>(green, 0, 255)),
      static_cast<uint8_t>(ait::clamp<FloatType>(blue, 0, 255)), 255);
  return color;
}

ViewpointPlanner::FloatType ViewpointPlanner::decodeDepthValue(const ait::Color4<uint8_t>& color) const {
  const FloatType depth = color.r() * FloatType(256) + color.g() + color.b() / FloatType(256.);
  return depth;
}

ViewpointPlanner::FloatType ViewpointPlanner::decodeDepthValue(const QColor& pixel) const {
  const FloatType depth = pixel.red() * FloatType(256) + pixel.green() + pixel.blue() / FloatType(256.);
  return depth;
}

QImage ViewpointPlanner::convertEncodedDepthImageToRGB(
    const QImage& encoded_image, const FloatType min_depth, const FloatType max_depth) const {
  Eigen::MatrixXf depth_matrix(encoded_image.height(), encoded_image.width());
  for (int y = 0; y < encoded_image.height(); ++y) {
    for (int x = 0; x < encoded_image.width(); ++x) {
      depth_matrix(y, x) = decodeDepthValue(QColor(encoded_image.pixel(x, y)));
    }
  }
  QImage rgb_image(encoded_image.width(), encoded_image.height(), QImage::Format_ARGB32);
  ait::ColorMapHot<FloatType> cmap;
  for (int y = 0; y < encoded_image.height(); ++y) {
    for (int x = 0; x < encoded_image.width(); ++x) {
      const FloatType depth = ait::clamp(depth_matrix(y, x), min_depth, max_depth);
      const ait::Color3<FloatType> color = cmap.map(ait::normalize(depth, min_depth, max_depth));
      const QColor pixel(
          ait::clamp<int>(255 * color.r(), 0, 255),
          ait::clamp<int>(255 * color.g(), 0, 255),
          ait::clamp<int>(255 * color.b(), 0, 255));
      rgb_image.setPixel(x, y, pixel.rgba());
    }
  }
  return rgb_image;
}

#endif
