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
  GLuint color_buffer_id_;
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
      const glm::vec3 &color,
      std::vector<T> &vertex_buffer,
      std::vector<U> &color_buffer) const
  {
    vertex_buffer.push_back(vertex.x());
    vertex_buffer.push_back(vertex.y());
    vertex_buffer.push_back(vertex.z());
    color_buffer.push_back(color.r);
    color_buffer.push_back(color.g);
    color_buffer.push_back(color.b);
  }

  template <typename T, typename U, typename V>
  void putTriangleIntoVertexBuffer(
      const octomap::point3d &vertex1,
      const octomap::point3d &vertex2,
      const octomap::point3d &vertex3,
      const glm::vec3 &color,
      std::vector<T> &vertex_buffer,
      std::vector<U> &color_buffer,
      std::vector<V> &index_buffer) const
  {
    index_buffer.push_back(vertex_buffer.size() / 3);
    putVertexIntoVertexBuffer(vertex1, color, vertex_buffer, color_buffer);
    index_buffer.push_back(vertex_buffer.size() / 3);
    putVertexIntoVertexBuffer(vertex2, color, vertex_buffer, color_buffer);
    index_buffer.push_back(vertex_buffer.size() / 3);
    putVertexIntoVertexBuffer(vertex3, color, vertex_buffer, color_buffer);
  }

  template <typename T, typename U, typename V>
  void putVoxelTrianglesIntoVertexBuffer(
      const octomap::point3d &center,
      double size,
      std::vector<T> &vertex_buffer,
      std::vector<U> &color_buffer,
      std::vector<V> &index_buffer) const;
};

}
}
