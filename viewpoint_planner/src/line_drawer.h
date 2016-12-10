//==================================================
// line_drawer.h
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

struct OGLLineData
{
    OGLVertexData vertex1;
    OGLVertexData vertex2;
};

inline std::ostream& operator<<(std::ostream& out, const OGLLineData& line) {
    out << "[ " << line.vertex1 << " " << line.vertex2 << " ]";
    return out;
}

class LineDrawer
{
public:
    LineDrawer()
    : num_lines_(0) {
    }

    ~LineDrawer() {
        clear();
    }

    void clear() {
        num_lines_ = 0;
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
        program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/lines.v.glsl");
        program_.addShaderFromSourceFile(QOpenGLShader::Geometry, ":/shaders/lines.g.glsl");
        program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/lines.f.glsl");
        program_.link();

        vao_.create();
        vbo_.create();
    }

    void upload(const std::vector<OGLLineData>& line_data) {
        num_lines_ = line_data.size();

        program_.bind();
        vao_.bind();
        vbo_.bind();

        vbo_.setUsagePattern(QOpenGLBuffer::DynamicDraw);
        vbo_.allocate(line_data.data(), line_data.size() * sizeof(OGLLineData));

        program_.enableAttributeArray(0);
        program_.setAttributeBuffer(0, GL_FLOAT, 0, 3, sizeof(OGLVertexData));


        program_.enableAttributeArray(1);
        program_.setAttributeBuffer(1, GL_FLOAT, 3 * sizeof(float), 4, sizeof(OGLVertexData));

        vbo_.release();
        vao_.release();
        program_.release();
    }

    void draw(const QMatrix4x4& pmv_matrix, const int width, const int height, const float line_width) {
        if (num_lines_ == 0) {
            return;
        }

        program_.bind();
        vao_.bind();

        program_.setUniformValue("u_pmv_matrix", pmv_matrix);
        program_.setUniformValue("u_inv_viewport", QVector2D(1.0f / width, 1.0f / height));
        program_.setUniformValue("u_line_width", line_width);

        glDrawArrays(GL_LINES, 0, 2 * num_lines_);

        vao_.release();
        program_.release();
    }

private:
    size_t num_lines_;
    QOpenGLVertexArrayObject vao_;
    QOpenGLBuffer vbo_;
    QOpenGLShaderProgram program_;
};
