//==================================================
// viewpoint_planner_window.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include "viewpoint_planner.h"

#include <thread>
#include <QOpenGLWidget>
#include <QMatrix4x4>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <QWheelEvent>
#include <QBasicTimer>
#include <QtMath>
#include <cmath>
#include <QTime>
#include <QGLViewer/qglviewer.h>
#include "qt_utils.h"
#include "octomap_renderer.h"

class ViewpointPlannerWindow : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

    const float DEFAULT_MODEL_SCALE = 0.1f;
    const float ARCBALL_ROTATION_THRESHOLD = 1e-6f;

public:
    ViewpointPlannerWindow(ViewpointPlanner* planner_ptr, QWidget *parent = nullptr)
    : QOpenGLWidget(parent), planner_ptr_(planner_ptr), octomap_renderer_(planner_ptr_->getOctomap()) {
        QSurfaceFormat glFormat;
        glFormat.setVersion(3, 4);
        glFormat.setProfile(QSurfaceFormat::CoreProfile);
        glFormat.setSamples(4);
        setFormat(glFormat);
        resize(1024, 768);
    }

    virtual ~ViewpointPlannerWindow()
    {
        // Make sure the context is current when deleting textures and buffers.
        makeCurrent();
        doneCurrent();
    }

    void start() {
    }

    void join() {
    }

protected:
//    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE {
////        // Save mouse press position
////        mousePressPosition = QVector2D(e->localPos());
//    }
//    void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE {
////        // Mouse release position - mouse press position
////        QVector2D diff = QVector2D(e->localPos()) - mousePressPosition;
////
////        // Rotation axis is perpendicular to the mouse position difference
////        // vector
////        QVector3D n = QVector3D(diff.y(), diff.x(), 0.0).normalized();
////
////        // Accelerate angular speed relative to the length of the mouse sweep
////        qreal acc = diff.length() / 100.0;
////
////        // Calculate new rotation axis as weighted sum
////        rotationAxis = (rotationAxis * angularSpeed + n * acc).normalized();
////
////        // Increase angular speed
////        angularSpeed += acc;
//    }

    void timerEvent(QTimerEvent *e) Q_DECL_OVERRIDE {
        update();
    }

    void initializeGL() Q_DECL_OVERRIDE {
        initializeOpenGLFunctions();
        octomap_renderer_.initializeGL();

        glClearColor(0.3f, 0.3f, 0.3f, 0.0f);

        initShaders();
//        initTextures();

        // Enable multi-sampling
        glEnable(GL_MULTISAMPLE);

        // Enable depth buffer
        glEnable(GL_DEPTH_TEST);

        // Enable back face culling
        glEnable(GL_CULL_FACE);

//        geometries = new GeometryEngine;
//
//        // Use QBasicTimer because its faster than QTimer
        timer.start(12, this);
    }

    void initShaders() {
        // Compile vertex shader
        if (!octomap_program_.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/octomap_shader.vertex")) {
            close();
        }

        // Compile fragment shader
        if (!octomap_program_.addShaderFromSourceFile(QOpenGLShader::Geometry, ":/octomap_shader.geometry")) {
            close();
        }

        // Compile fragment shader
        if (!octomap_program_.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/octomap_shader.fragment")) {
            close();
        }

        // Link shader pipeline
        if (!octomap_program_.link()) {
            close();
        }
    }

    void resetView(bool reset_scale=true) {
        // Projection matrix: 110° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
        qreal aspect = 1.0f;
        if (width() > 0) {
          aspect = static_cast<qreal>(width()) / height();
        }
        const qreal zNear = 0.1;
        const qreal zFar = 100.0;
        const qreal fov = 45.0f;
        projection_.perspective(fov, aspect, zNear, zFar);
        // Camera matrix
        view_.lookAt(
                QVector3D(20, 0, 20), // Camera location
                QVector3D(0, 0, 0), // Scene center
                QVector3D(0, 0, 1)); // Up direction
        if (reset_scale) {
          // Model matrix : an identity matrix (model will be at the origin)
          view_ *= QMatrix4x4() * DEFAULT_MODEL_SCALE;
        }
    }

    void resizeGL(int w, int h) Q_DECL_OVERRIDE {
        // Calculate aspect ratio
        qreal aspect = qreal(w) / qreal(h ? h : 1);
//
        // Set near plane to 3.0, far plane to 7.0, field of view 45 degrees
//        const qreal zNear = 3.0, zFar = 7.0, fov = 45.0;
//
        // Reset projection
        projection_.setToIdentity();
//
        const qreal zNear = 0.1;
        const qreal zFar = 100.0;
        const qreal fov = 45.0f;

        // Set perspective projection
        projection_.perspective(fov, aspect, zNear, zFar);

        // Camera matrix
        view_.lookAt(
                QVector3D(20, 0, 20), // Camera location
                QVector3D(0, 0, 0), // Scene center
                QVector3D(0, 0, 1)); // Up direction
    }

    void paintGL() Q_DECL_OVERRIDE {
        //        updateMatricesFromInputs();

        // Clear color and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glFrontFace(GL_CCW);
        glCullFace(GL_BACK);
        glEnable(GL_CULL_FACE);
        //    glDisable(GL_CULL_FACE);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        // Enable alpha blendingglEnable( GL_BLEND )
        glEnable(GL_BLEND);
        glBlendEquation(GL_FUNC_ADD);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//        render();
//        for (const auto& object : objects) {
//        object.render(view_, projection_, width_, height_);
//        }
//        for (const auto& object : objects2) {
//        object.render(view_, projection_, width_, height_);
//        }

        octomap_program_.bind();
        octomap_program_.setUniformValue("viewportWidth", width());
        octomap_program_.setUniformValue("viewportHeight", height());
        QMatrix4x4 mvp = projection_ * view_ * model_;
        octomap_program_.setUniformValue("MVP", mvp);
        octomap_program_.setUniformValue("M", model_);
        octomap_program_.setUniformValue("V", view_);
        octomap_renderer_.render();
    }

    void wheelEvent(QWheelEvent *event) Q_DECL_OVERRIDE {
        QPoint numDegrees = event->angleDelta() / 8;
        float yoffset = numDegrees.y() / 2.0f;
//        std::cout << "yoffset: " << yoffset << std::endl;

        float view_scale = view_.column(0).length();
//        std::cout << "View scale: " << view_scale << std::endl;
        float translate_speed = 0.1f / view_scale / view_scale;
//        std::cout << "translate_speed: " << translate_speed << std::endl;
//        std::cout << "view: " << view_ << std::endl;
        QMatrix4x4 inv_rot = view_.inverted();
        inv_rot.setColumn(3, QVector4D());
        inv_rot(3, 3) = 1;
        QVector3D translation_vec = inv_rot * QVector3D(0, 0, translate_speed * yoffset);
//        std::cout << "translation_vec:" << translation_vec << std::endl;
        view_.translate(translation_vec);
//        std::cout << "translated view: " << view_ << std::endl;

        event->accept();
    }

    void mousePressEvent(QMouseEvent *e) Q_DECL_OVERRIDE {
        if (e->button() == Qt::LeftButton) {
            left_button_pressed_ = true;
        }
        if (e->button() == Qt::RightButton) {
            right_button_pressed_ = true;
        }
        last_cursor_press_pos = e->localPos();
        start_arcball_vector = getArcballVector(e->localPos().x(), e->localPos().y());
        start_view_matrix = view_;
    }

    void mouseReleaseEvent(QMouseEvent *e) Q_DECL_OVERRIDE {
        if (e->button() == Qt::LeftButton) {
            left_button_pressed_ = false;
        }
        if (e->button() == Qt::RightButton) {
            right_button_pressed_ = false;
        }
    }

    void mouseMoveEvent(QMouseEvent *e) Q_DECL_OVERRIDE {
        QPointF d_cursor_pos = e->localPos() - last_cursor_press_pos;
        qreal d_cursor_pos_x = d_cursor_pos.x();
        qreal d_cursor_pos_y = d_cursor_pos.y();
        bool left_button_pressed = e->buttons() & Qt::LeftButton;
        bool right_button_pressed = e->buttons() & Qt::RightButton;
        updateMatricesFromInputs(e->localPos().x(), e->localPos().y(), d_cursor_pos_x, d_cursor_pos_y, left_button_pressed, right_button_pressed);
        last_cursor_press_pos = e->localPos();
    }

    QVector3D getArcballVector(qreal cursor_pos_x, qreal cursor_pos_y) const {
        QVector3D point(
            2.0 * cursor_pos_x / width() - 1.0,
            2.0 * cursor_pos_y / height() - 1.0,
            0
        );
        point.setY(-point.y());
        float dist_squared = point.x() * point.x() + point.y() * point.y();
        if (dist_squared <= 1.0f) {
            point.setZ(std::sqrt(1.0f - dist_squared));
        }
        else {
            std::cout << "arcball vector length: " << point.length() << std::endl;
            std::cout << "out of reach" << std::endl;
            point.normalize();
        }
        return point;
    }

    void updateMatricesFromInputs(qreal cursor_pos_x, qreal cursor_pos_y, qreal d_cursor_pos_x, qreal d_cursor_pos_y, bool left_button_pressed, bool right_button_pressed) {
      // TODO: Should be done per frame for everyone
      static QTime time;
      if (time.isNull()) {
          time.start();
      }
      double deltaTime = time.elapsed() / 1000.0;
      time.restart();

      // TODO: Clean this up
      if (right_button_pressed && !left_button_pressed) {
    //    std::cout << "d_cursor_pos_x: " << d_cursor_pos_x << ", d_cursor_pos_y: " << d_cursor_pos_y << std::endl;
        d_cursor_pos_x /= width();
        d_cursor_pos_y /= height();

        // TODO: Should be a member
        float view_scale = view_.column(0).length();
        float translate_speed = 5.0f / view_scale / view_scale;
        QMatrix4x4 inv_rot = view_.inverted();
        inv_rot.setColumn(3, QVector4D());
        inv_rot(3, 3) = 1;
        QVector3D translation_vec = inv_rot * QVector3D(d_cursor_pos_x, -d_cursor_pos_y, 0) * translate_speed;
//        std::cout << "translation_vec:" << translation_vec << std::endl;
        view_.translate(translation_vec);
    //    std::cout << "translation_vec: " << translation_vec << std::endl;
      }
      else if (left_button_pressed && !right_button_pressed) {
        // TODO: still some bug here
        QVector3D arcball_vector = getArcballVector(cursor_pos_x, cursor_pos_y);
        if ((arcball_vector - start_arcball_vector).length() > ARCBALL_ROTATION_THRESHOLD) {
          std::cout << "start arcball_vector: " << start_arcball_vector << std::endl;
          std::cout << "arcball_vector: " << arcball_vector << std::endl;
          float angle = std::acos(std::min(1.0f, QVector3D::dotProduct(start_arcball_vector, arcball_vector)));
          QVector3D rotation_axis = QVector3D::crossProduct(start_arcball_vector, arcball_vector);
          std::cout << "rotation_axis: " << rotation_axis << std::endl;
          QVector3D view_rotation_axis = start_view_matrix.transposed() * rotation_axis;
          QMatrix4x4 new_view = start_view_matrix;
          new_view.rotate(qRadiansToDegrees(angle), view_rotation_axis);
          view_ = new_view;
        }
      }
    }

private:
    bool left_button_pressed_ = false;
    bool right_button_pressed_ = false;
    QPointF last_cursor_press_pos;
    QVector3D start_arcball_vector;
    QMatrix4x4 start_view_matrix;

    QVector3D getArcBallVector(int x, int y) {
       QVector3D pt = QVector3D(2.0 * x / width() - 1.0, 2.0 * y / height() - 1.0 , 0);
       pt.setY(pt.y() * -1);

       // compute z-coordinates
       float xySquared = pt.x() * pt.x() + pt.y() * pt.y();

       if(xySquared <= 1.0) {
           pt.setZ(std::sqrt(1.0 - xySquared));
       }
       else {
           pt.normalize();
       }

       return pt;

    }

    void run() {
    }

    ViewpointPlanner* planner_ptr_;
    OctomapRenderer octomap_renderer_;
    QOpenGLShaderProgram octomap_program_;

    QMatrix4x4 projection_;
    QMatrix4x4 view_;
    QMatrix4x4 model_;

    QBasicTimer timer;
};
*/
