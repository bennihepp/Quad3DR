//==================================================
// octomap_renderer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <vector>
#include <memory>
#include <octomap/octomap.h>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QOpenGLFunctions>

class OctomapRenderer : protected QOpenGLFunctions
{
  const octomap::OcTree* octree_ptr_;

  QOpenGLVertexArrayObject vertex_array_;
  QOpenGLBuffer vertex_buffer_;
  QOpenGLBuffer size_buffer_;
  QOpenGLBuffer index_buffer_;
  bool gl_initialized_;

public:
  OctomapRenderer();
  OctomapRenderer(const octomap::OcTree* octree_ptr);
  virtual ~OctomapRenderer();
  void initializeGL();

  void setOctomap(const octomap::OcTree* octree_ptr);
  void render();

protected:
  void initRenderBuffers();
  void clearRenderBuffers();

  template <typename T, typename U>
  void putVertexIntoVertexBuffer(
      const octomap::point3d &vertex,
      const float size,
      std::vector<T> &vertex_buffer,
      std::vector<U> &size_buffer) const
  {
    vertex_buffer.push_back(vertex.x());
    vertex_buffer.push_back(vertex.y());
    vertex_buffer.push_back(vertex.z());
    size_buffer.push_back(size);
  }

  template <typename T, typename U, typename V>
  void putVoxelIntoVertexBuffer(
      const octomap::point3d &center,
      double size,
      std::vector<T> &vertex_buffer,
      std::vector<U> &color_buffer,
      std::vector<V> &index_buffer) const;
};

template <typename T, typename U, typename V>
void OctomapRenderer::putVoxelIntoVertexBuffer(const octomap::point3d &center, double size, std::vector<T> &vertex_buffer, std::vector<U> &size_buffer, std::vector<V> &index_buffer) const
{
  index_buffer.push_back(vertex_buffer.size() / 3);
  vertex_buffer.push_back(center.x());
  vertex_buffer.push_back(center.y());
  vertex_buffer.push_back(center.z());
  size_buffer.push_back(size);
}
