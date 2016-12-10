//==================================================
// octomap_renderer.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================

#include "octomap_renderer.h"
#include <QtGui/qopengl.h>
#include <QtGui/qwindow.h>

OctomapRenderer::OctomapRenderer()
: octree_ptr_(nullptr), index_buffer_(QOpenGLBuffer::IndexBuffer), gl_initialized_(false)
{
}

OctomapRenderer::OctomapRenderer(const octomap::OcTree* octree_ptr)
: index_buffer_(QOpenGLBuffer::IndexBuffer), gl_initialized_(false)
{
    setOctomap(octree_ptr);
}

OctomapRenderer::~OctomapRenderer()
{
    clearRenderBuffers();
}

void OctomapRenderer::clearRenderBuffers()
{
    if (gl_initialized_) {
        index_buffer_.destroy();
        size_buffer_.destroy();
        vertex_buffer_.destroy();
        vertex_array_.destroy();
    }
}

void OctomapRenderer::setOctomap(const octomap::OcTree* octree_ptr)
{
    if (octree_ptr_ != nullptr) {
        clearRenderBuffers();
    }
    octree_ptr_ = octree_ptr;
    initRenderBuffers();
}

void OctomapRenderer::initializeGL()
{
    initializeOpenGLFunctions();
    gl_initialized_ = true;
    if (octree_ptr_ != nullptr) {
        initRenderBuffers();
    }
}

void OctomapRenderer::initRenderBuffers()
{
    if (!gl_initialized_) {
        return;
    }
    std::vector<GLfloat> vertex_buffer;
    std::vector<GLfloat> size_buffer;
    std::vector<GLuint> index_buffer;

//    octomap::point3d bbx_min(-10, -10, -10);
//    octomap::point3d bbx_max(+10, +10, +10);
//    for (auto it = octree_ptr_->begin_leafs_bbx(bbx_min, bbx_max); it != octree_ptr_->end_leafs_bbx(); ++it) {
    for (auto it = octree_ptr_->begin_leafs(); it != octree_ptr_->end_leafs(); ++it) {
        const octomap::point3d &coord = it.getCoordinate();
        double size = it.getSize();
        //    if (size < 0.2)
        //      continue;
        double occupancy = it->getOccupancy();
        if (occupancy > 0.5) {
            putVoxelIntoVertexBuffer(coord, size, vertex_buffer, size_buffer, index_buffer);
        }
    }
    std::cout << "Map has " << (index_buffer.size()) << " voxels and "
            << (vertex_buffer.size() / 3) << " vertices" << std::endl;

    // Setup vertex arrays
    vertex_array_.create();
    vertex_array_.bind();

    // Vertex attribute buffer
    vertex_buffer_.create();
    vertex_buffer_.bind();
    vertex_buffer_.allocate(&vertex_buffer[0], sizeof(GLfloat) * vertex_buffer.size());
    glVertexAttribPointer(
        0,                  // attribute 0. No particular reason for 0, but must match the layout in the shader.
        3,                  // size
        GL_FLOAT,           // type
        false,           // normalized?
        0,                  // stride
        (void*)0            // array buffer offset
    );
    glEnableVertexAttribArray(0);

    // Color attribute buffer
    size_buffer_.create();
    size_buffer_.bind();
    size_buffer_.allocate(&size_buffer[0], sizeof(GLfloat) * size_buffer.size());
    glVertexAttribPointer(
        1,                                // attribute. No particular reason for 1, but must match the layout in the shader.
        1,                                // size
        GL_FLOAT,                         // type
        false,                         // normalized?
        0,                                // stride
        (void*)0                          // array buffer offset
    );
    glEnableVertexAttribArray(1);

    // Index buffer
    index_buffer_.create();
    index_buffer_.bind();
    index_buffer_.allocate(&index_buffer[0], sizeof(GLuint) * index_buffer.size());

    vertex_array_.release();
}

void OctomapRenderer::render()
{
    if (octree_ptr_) {
        vertex_array_.bind();
//    glEnableVertexAttribArray(0);
//    glEnableVertexAttribArray(1);
//    glDrawElements(GL_TRIANGLES, index_buffer_size_, GL_UNSIGNED_INT, nullptr);
        glDrawElements(GL_POINTS, index_buffer_.size(), GL_UNSIGNED_INT, nullptr);
//    glDrawArrays(GL_TRIANGLES, 0, vertex_buffer.size());
//    glDisableVertexAttribArray(1);
//    glDisableVertexAttribArray(0);
  }
}
