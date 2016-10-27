//==================================================
// octomap_renderer.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Sep 21, 2016
//==================================================

#include <quad_planner/rendering/octomap_renderer.h>
#include <quad_planner/rendering/glm.h>
#include <quad_planner/rendering/color.h>

using namespace glm;
using namespace quad_planner::rendering;


OctomapRenderer::OctomapRenderer()
: vertex_array_id_(0), vertex_buffer_id_(0),
  size_buffer_id_(0), index_buffer_id_(0),
  index_buffer_size_(0)
{
}

OctomapRenderer::OctomapRenderer(std::shared_ptr<const octomap::OcTree> octree_ptr)
: vertex_array_id_(0), vertex_buffer_id_(0),
  size_buffer_id_(0), index_buffer_id_(0),
  index_buffer_size_(0)
{
  setOctomap(octree_ptr);
}

OctomapRenderer::~OctomapRenderer()
{
  if (octree_ptr_)
  {
    clearRenderBuffers();
  }
}

void OctomapRenderer::setOctomap(std::shared_ptr<const octomap::OcTree> octree_ptr)
{
  if (octree_ptr_)
  {
    clearRenderBuffers();
  }
  octree_ptr_ = octree_ptr;
  initRenderBuffers();
}

//void OctomapRenderer::initRenderBuffers()
//{
//  std::vector<GLfloat> vertex_buffer;
//  std::vector<GLfloat> color_buffer;
//  std::vector<GLuint> index_buffer;
////  vertex_buffer_data.push_back(-1.0f);
////  vertex_buffer_data.push_back(-1.0f);
////  vertex_buffer_data.push_back(0.0f);
////  vertex_buffer_data.push_back(1.0f);
////  vertex_buffer_data.push_back(-0.8f);
////  vertex_buffer_data.push_back(0.0f);
////  vertex_buffer_data.push_back(-0.3f);
////  vertex_buffer_data.push_back(1.0f);
////  vertex_buffer_data.push_back(0.0f);
////
////  octomap::point3d coord(0, 0, 0);
////  double size = 0.1;
////  putVoxelTrianglesIntoVertexBuffer(coord, size, vertex_buffer_data, color_buffer_data, index_buffer_data);
////
////  octomap::point3d vertex1(-1.0f, -1.0f, 0.0f);
////  octomap::point3d vertex2(1.0f, -0.8f, 0.0f);
////  octomap::point3d vertex3(-0.3f, 1.0f, 0.0f);
////  putTriangleIntoVertexBuffer(vertex1, vertex2, vertex3, ColorGlm::red(), vertex_buffer, color_buffer, index_buffer);
////
//  octomap::point3d bbx_min(-10, -10, -10);
//  octomap::point3d bbx_max(+10, +10, +10);
//  for (auto it = octree_ptr_->begin_leafs_bbx(bbx_min, bbx_max); it != octree_ptr_->end_leafs_bbx(); ++it)
//  {
//    const octomap::point3d &coord = it.getCoordinate();
//    double size = it.getSize();
////    if (size < 0.2)
////      continue;
//    double occupancy = it->getOccupancy();
//    if (occupancy > 0.5)
//    {
//      putVoxelTrianglesIntoVertexBuffer(coord, size, vertex_buffer, color_buffer, index_buffer);
//    }
//  }
//  std::cout << "Mesh has " << (index_buffer.size() / 3) << " triangles and "
//            << (vertex_buffer.size() / 3) << " vertices and "
//            << vertex_buffer.size() << " values" << std::endl;
//
//  // Setup vertex arrays
//  glGenVertexArrays(1, &vertex_array_id_);
//  glBindVertexArray(vertex_array_id_);
//
//  // Vertex attribute buffer
//  glGenBuffers(1, &vertex_buffer_id_);
//  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id_);
//  glBufferData(GL_ARRAY_BUFFER, vertex_buffer.size() * sizeof(GLfloat), &vertex_buffer[0], GL_STATIC_DRAW);
//  glVertexAttribPointer(
//     0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
//     3,                  // size
//     GL_FLOAT,           // type
//     GL_FALSE,           // normalized?
//     0,                  // stride
//     (void*)0            // array buffer offset
//  );
//  glEnableVertexAttribArray(0);
//
//  // Color attribute buffer
//  glGenBuffers(1, &color_buffer_id_);
//  glBindBuffer(GL_ARRAY_BUFFER, color_buffer_id_);
//  glBufferData(GL_ARRAY_BUFFER, color_buffer.size() * sizeof(GLfloat), &color_buffer[0], GL_STATIC_DRAW);
//  glVertexAttribPointer(
//      1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
//      3,                                // size
//      GL_FLOAT,                         // type
//      GL_FALSE,                         // normalized?
//      0,                                // stride
//      (void*)0                          // array buffer offset
//  );
//  glEnableVertexAttribArray(1);
//
//  // Index buffer
//  glGenBuffers(1, &index_buffer_id_);
//  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_id_);
//  glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer.size() * sizeof(GLuint), &index_buffer[0], GL_STATIC_DRAW);
//  index_buffer_size_ = index_buffer.size();
//
//  glBindVertexArray(0);
//}

void OctomapRenderer::initRenderBuffers()
{
  std::vector<GLfloat> vertex_buffer;
  std::vector<GLfloat> size_buffer;
  std::vector<GLuint> index_buffer;

  octomap::point3d bbx_min(-10, -10, -10);
  octomap::point3d bbx_max(+10, +10, +10);
  for (auto it = octree_ptr_->begin_leafs_bbx(bbx_min, bbx_max); it != octree_ptr_->end_leafs_bbx(); ++it)
  {
    const octomap::point3d &coord = it.getCoordinate();
    double size = it.getSize();
//    if (size < 0.2)
//      continue;
    double occupancy = it->getOccupancy();
    if (occupancy > 0.5)
    {
      putVoxelIntoVertexBuffer(coord, size, vertex_buffer, size_buffer, index_buffer);
    }
  }
  std::cout << "Mesh has " << (index_buffer.size() / 3) << " triangles and "
            << (vertex_buffer.size() / 3) << " vertices and "
            << vertex_buffer.size() << " values" << std::endl;

  // Setup vertex arrays
  glGenVertexArrays(1, &vertex_array_id_);
  glBindVertexArray(vertex_array_id_);

  // Vertex attribute buffer
  glGenBuffers(1, &vertex_buffer_id_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, vertex_buffer.size() * sizeof(GLfloat), &vertex_buffer[0], GL_STATIC_DRAW);
  glVertexAttribPointer(
     0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
     3,                  // size
     GL_FLOAT,           // type
     GL_FALSE,           // normalized?
     0,                  // stride
     (void*)0            // array buffer offset
  );
  glEnableVertexAttribArray(0);

  // Color attribute buffer
  glGenBuffers(1, &size_buffer_id_);
  glBindBuffer(GL_ARRAY_BUFFER, size_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, size_buffer.size() * sizeof(GLfloat), &size_buffer[0], GL_STATIC_DRAW);
  glVertexAttribPointer(
      1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
      1,                                // size
      GL_FLOAT,                         // type
      GL_FALSE,                         // normalized?
      0,                                // stride
      (void*)0                          // array buffer offset
  );
  glEnableVertexAttribArray(1);

  // Index buffer
  glGenBuffers(1, &index_buffer_id_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_id_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, index_buffer.size() * sizeof(GLuint), &index_buffer[0], GL_STATIC_DRAW);
  index_buffer_size_ = index_buffer.size();

  glBindVertexArray(0);
}

void OctomapRenderer::clearRenderBuffers()
{
  glDeleteBuffers(1, &index_buffer_id_);
  glDeleteBuffers(1, &size_buffer_id_);
  glDeleteBuffers(1, &vertex_buffer_id_);
  glDeleteVertexArrays(1, &vertex_array_id_);
  index_buffer_size_ = 0;
  index_buffer_id_ = 0;
  size_buffer_id_ = 0;
  vertex_buffer_id_ = 0;
  vertex_array_id_ = 0;
}

void OctomapRenderer::render() const
{
  if (octree_ptr_)
  {
    glBindVertexArray(vertex_array_id_);
//    glEnableVertexAttribArray(0);
//    glEnableVertexAttribArray(1);
//    glDrawElements(GL_TRIANGLES, index_buffer_size_, GL_UNSIGNED_INT, nullptr);
    glDrawElements(GL_POINTS, index_buffer_size_, GL_UNSIGNED_INT, nullptr);
//    glDrawArrays(GL_TRIANGLES, 0, vertex_buffer.size());
//    glDisableVertexAttribArray(1);
//    glDisableVertexAttribArray(0);
  }
}
