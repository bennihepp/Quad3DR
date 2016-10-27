//==================================================
// octomap_renderer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 21, 2016
//==================================================

#pragma once

#include <vector>
#include <memory>
#include <GL/glew.h>
#include <octomap/octomap.h>
#include <quad_planner/rendering/linalg.h>
#include <quad_planner/rendering/render_object.h>

namespace quad_planner
{
namespace rendering
{

class OctomapRenderer : public RenderObject
{
  std::shared_ptr<const octomap::OcTree> octree_ptr_;

  GLuint vertex_array_id_;
  GLuint vertex_buffer_id_;
  GLuint size_buffer_id_;
  GLuint index_buffer_id_;
  GLsizei index_buffer_size_;

public:
  OctomapRenderer();
  OctomapRenderer(std::shared_ptr<const octomap::OcTree> octree_ptr);
  virtual ~OctomapRenderer();

  void setOctomap(std::shared_ptr<const octomap::OcTree> octree_ptr);
  void render() const;

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

//  template <typename T, typename U>
//  void putVertexIntoVertexBuffer(
//      const octomap::point3d &vertex,
//      const glm::vec3 &color,
//      std::vector<T> &vertex_buffer,
//      std::vector<U> &color_buffer) const
//  {
//    vertex_buffer.push_back(vertex.x());
//    vertex_buffer.push_back(vertex.y());
//    vertex_buffer.push_back(vertex.z());
//    color_buffer.push_back(color.r);
//    color_buffer.push_back(color.g);
//    color_buffer.push_back(color.b);
//  }

//  template <typename T, typename U, typename V>
//  void putTriangleIntoVertexBuffer(
//      const octomap::point3d &vertex1,
//      const octomap::point3d &vertex2,
//      const octomap::point3d &vertex3,
//      const glm::vec3 &color,
//      std::vector<T> &vertex_buffer,
//      std::vector<U> &color_buffer,
//      std::vector<V> &index_buffer) const
//  {
//    index_buffer.push_back(vertex_buffer.size() / 3);
//    putVertexIntoVertexBuffer(vertex1, color, vertex_buffer, color_buffer);
//    index_buffer.push_back(vertex_buffer.size() / 3);
//    putVertexIntoVertexBuffer(vertex2, color, vertex_buffer, color_buffer);
//    index_buffer.push_back(vertex_buffer.size() / 3);
//    putVertexIntoVertexBuffer(vertex3, color, vertex_buffer, color_buffer);
//  }

//  template <typename T, typename U, typename V>
//  void putVoxelTrianglesIntoVertexBuffer(
//      const octomap::point3d &center,
//      double size,
//      std::vector<T> &vertex_buffer,
//      std::vector<U> &color_buffer,
//      std::vector<V> &index_buffer) const;

  template <typename T, typename U, typename V>
  void putVoxelIntoVertexBuffer(
      const octomap::point3d &center,
      double size,
      std::vector<T> &vertex_buffer,
      std::vector<U> &color_buffer,
      std::vector<V> &index_buffer) const;
};

//template <typename T, typename U, typename V>
//void OctomapRenderer::putVoxelTrianglesIntoVertexBuffer(const octomap::point3d &center, double size, std::vector<T> &vertex_buffer, std::vector<U> &color_buffer, std::vector<V> &index_buffer) const
//{
//  octomap::point3d extent(size, size, size);
//  octomap::point3d extent_x(size, 0, 0);
//  octomap::point3d extent_y(0, size, 0);
//  octomap::point3d extent_z(0, 0, size);
//  octomap::point3d corner1 = center - extent * 0.5f;
//  octomap::point3d corner2 = corner1 + extent_y;
//  octomap::point3d corner3 = corner2 + extent_x;
//  octomap::point3d corner4 = corner3 - extent_y;
//  octomap::point3d corner5 = corner4 + extent_z;
//  octomap::point3d corner6 = corner3 + extent_z;
//  octomap::point3d corner7 = corner2 + extent_z;
//  octomap::point3d corner8 = corner1 + extent_z;
//
//  glm::vec3 color_front_1 = ColorGlm::green();
//  glm::vec3 color_front_2 = ColorGlm::cyan();
//  glm::vec3 color_back_1 = ColorGlm::red();
//  glm::vec3 color_back_2 = ColorGlm::magenta();
//  glm::vec3 color_left_1 = ColorGlm::orange();
//  glm::vec3 color_left_2 = ColorGlm::blue();
//  glm::vec3 color_right_1 = ColorGlm::yellow();
//  glm::vec3 color_right_2 = ColorGlm::violet();
//  glm::vec3 color_bottom_1 = ColorGlm::cyan();
//  glm::vec3 color_bottom_2 = ColorGlm::red();
//  glm::vec3 color_top_1 = ColorGlm::orange();
//  glm::vec3 color_top_2 = ColorGlm::green();
//
//  // Front side of voxel
//  putTriangleIntoVertexBuffer(corner5, corner7, corner8, color_front_1, vertex_buffer, color_buffer, index_buffer);
//  putTriangleIntoVertexBuffer(corner5, corner6, corner7, color_front_2, vertex_buffer, color_buffer, index_buffer);
//  // Back side of voxel
//  putTriangleIntoVertexBuffer(corner1, corner3, corner4, color_back_1, vertex_buffer, color_buffer, index_buffer);
//  putTriangleIntoVertexBuffer(corner1, corner2, corner3, color_back_2, vertex_buffer, color_buffer, index_buffer);
//  // Left side of voxel
//  putTriangleIntoVertexBuffer(corner2, corner1, corner8, color_left_1, vertex_buffer, color_buffer, index_buffer);
//  putTriangleIntoVertexBuffer(corner2, corner8, corner7, color_left_2, vertex_buffer, color_buffer, index_buffer);
//  // Right side of voxel
//  putTriangleIntoVertexBuffer(corner4, corner3, corner6, color_right_1, vertex_buffer, color_buffer, index_buffer);
//  putTriangleIntoVertexBuffer(corner4, corner6, corner5, color_right_2, vertex_buffer, color_buffer, index_buffer);
//  // Bottom side of voxel
//  putTriangleIntoVertexBuffer(corner4, corner8, corner1, color_bottom_1, vertex_buffer, color_buffer, index_buffer);
//  putTriangleIntoVertexBuffer(corner4, corner5, corner8, color_bottom_2, vertex_buffer, color_buffer, index_buffer);
//  // Top side of voxel
//  putTriangleIntoVertexBuffer(corner2, corner7, corner6, color_top_1, vertex_buffer, color_buffer, index_buffer);
//  putTriangleIntoVertexBuffer(corner2, corner6, corner3, color_top_2, vertex_buffer, color_buffer, index_buffer);
//}

template <typename T, typename U, typename V>
void OctomapRenderer::putVoxelIntoVertexBuffer(const octomap::point3d &center, double size, std::vector<T> &vertex_buffer, std::vector<U> &size_buffer, std::vector<V> &index_buffer) const
{
  index_buffer.push_back(vertex_buffer.size() / 3);
  vertex_buffer.push_back(center.x());
  vertex_buffer.push_back(center.y());
  vertex_buffer.push_back(center.z());
  size_buffer.push_back(size);
}

}
}
