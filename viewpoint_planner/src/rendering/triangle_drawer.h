//==================================================
// triangle_drawer.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 8, 2016
//==================================================
#pragma once

#include <array>
#include <QtOpenGL>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include "point_drawer.h"

struct OGLTriangleData
{
    OGLTriangleData() {}

    OGLTriangleData(const OGLVertexDataRGBA& vertex1, const OGLVertexDataRGBA& vertex2, const OGLVertexDataRGBA& vertex3)
    : vertex1(vertex1), vertex2(vertex2), vertex3(vertex3) {}

    OGLVertexDataRGBA vertex1;
    OGLVertexDataRGBA vertex2;
    OGLVertexDataRGBA vertex3;
};

inline std::ostream& operator<<(std::ostream& out, const OGLTriangleData& triangle) {
    out << "[ " << triangle.vertex1 << " " << triangle.vertex2 << " " << triangle.vertex3 << " ]";
    return out;
}

class TriangleDrawer
{
public:
    TriangleDrawer()
    : num_triangles_(0) {
    }

    ~TriangleDrawer() {
        clear();
    }

    void clear() {
        num_triangles_ = 0;
        if (vao_.isCreated()) {
            vao_.destroy();
        }
        if (vbo_.isCreated()) {
            vbo_.destroy();
        }
    }

    void init() {
        clear();

        if (program_.isLinked()) {
            program_.release();
            program_.removeAllShaders();
        }

        program_.create();
        program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/triangles.v.glsl");
        program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/triangles.f.glsl");
        program_.link();

        vao_.create();
        vbo_.create();
    }

    void upload(const std::vector<OGLTriangleData>& triangle_data) {
        num_triangles_ = triangle_data.size();

        program_.bind();
        vao_.bind();
        vbo_.bind();

        vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
        vbo_.allocate(triangle_data.data(), triangle_data.size() * sizeof(OGLTriangleData));

        program_.enableAttributeArray(0);
        program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexDataRGBA));

        program_.enableAttributeArray(1);
        program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 4, sizeof(OGLVertexDataRGBA));

        vbo_.release();
        vao_.release();
        program_.release();
    }

    void draw(const QMatrix4x4& pvm_matrix) {
        if (num_triangles_ == 0) {
            return;
        }

        program_.bind();
        vao_.bind();

        program_.setUniformValue("u_pvm_matrix", pvm_matrix);

        glDrawArrays(GL_TRIANGLES, 0, 3 * num_triangles_);

        vao_.release();
        program_.release();
    }

private:
    size_t num_triangles_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;
    QOpenGLShaderProgram program_;
};
