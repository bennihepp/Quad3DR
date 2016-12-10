//==================================================
// viewer_widget.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================

// This file is adapted from OctoMap.
// Original Copyright notice.
/*
 * This file is part of OctoMap - An Efficient Probabilistic 3D Mapping
 * Framework Based on Octrees
 * http://octomap.github.io
 *
 * Copyright (c) 2009-2014, K.M. Wurm and A. Hornung, University of Freiburg
 * All rights reserved. License for the viewer octovis: GNU GPL v2
 * http://www.gnu.org/licenses/old-licenses/gpl-2.0.txt
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see http://www.gnu.org/licenses/.
 */

#include "viewer_widget.h"
#include <cmath>
#include <manipulatedCameraFrame.h>
#include "pose.h"

using namespace std;

ViewerWidget::ViewerWidget(ViewerSettingsPanel* settings_panel, QWidget *parent)
    : QGLViewer(parent), initialized_(false), settings_panel_(settings_panel),
      octree_(nullptr), sparse_recon_(nullptr), draw_octree_(true), aspect_ratio_(-1) {
    QSizePolicy policy(QSizePolicy::Preferred, QSizePolicy::Preferred);
    policy.setHeightForWidth(true);
    setSizePolicy(policy);

    // Connect signals for settings panel
    connect(settings_panel_, SIGNAL(drawOctreeChanged(bool)), this, SLOT(setDrawOctree(bool)));
    connect(settings_panel_, SIGNAL(occupancyThresholdChanged(double)), this, SLOT(setOccupancyThreshold(double)));
    connect(settings_panel_, SIGNAL(voxelAlphaChanged(double)), this, SLOT(setVoxelAlpha(double)));
    connect(settings_panel_, SIGNAL(drawFreeVoxelsChanged(bool)), this, SLOT(setDrawFreeVoxels(bool)));
    connect(settings_panel_, SIGNAL(displayAxesChanged(bool)), this, SLOT(setDisplayAxes(bool)));
    connect(settings_panel_, SIGNAL(renderTreeDepthChanged(int)), this, SLOT(setRenderTreeDepth(int)));
    connect(settings_panel_, SIGNAL(drawSingleBinChanged(bool)), this, SLOT(setDrawSingleBin(bool)));
    connect(settings_panel_, SIGNAL(drawCamerasChanged(bool)), this, SLOT(setDrawCameras(bool)));
    connect(settings_panel_, SIGNAL(drawSparsePointsChanged(bool)), this, SLOT(setDrawSparsePoints(bool)));
    connect(settings_panel_, SIGNAL(refreshTree(void)), this, SLOT(refreshTree(void)));
    connect(settings_panel_, SIGNAL(useDroneCameraChanged(bool)), this, SLOT(setUseDroneCamera(bool)));
    connect(settings_panel_, SIGNAL(imagePoseChanged(ImageId)), this, SLOT(setImagePoseIndex(ImageId)));

    // Fill occupancy dropbox in settings panel
    double selected_occupancy_threshold = octree_drawer_.getOccupancyThreshold();
    settings_panel_->initializeOccupancyThresholds(octree_drawer_.getOccupancyBins());
    settings_panel_->selectOccupancyThreshold(selected_occupancy_threshold);
}

void ViewerWidget::init() {
    initialized_ = true;

    //    setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::AltModifier);
    //    setHandlerKeyboardModifiers(QGLViewer::FRAME, Qt::NoModifier);
    //    setHandlerKeyboardModifiers(QGLViewer::CAMERA, Qt::ControlModifier);
    setMouseTracking(true);

    // Restore previous viewer state.
    restoreStateFromFile();

    // Make camera the default manipulated frame.
    setManipulatedFrame(camera()->frame());
    // invert mousewheel (more like Blender)
    camera()->frame()->setWheelSensitivity(-1.0);

    // Light initialization:
    glEnable(GL_LIGHT0);

    // Directional light
    float pos[4] = {-1.0, 1.0, 1.0, 0.0};
    glLightfv(GL_LIGHT0, GL_POSITION, pos);

    // background color defaults to white
    this->setBackgroundColor( QColor(255,255,255) );
    this->qglClearColor( this->backgroundColor() );

    sparce_recon_drawer_.init();
    if (octree_ != nullptr) {
        showOctree(octree_);
    }
    if (sparse_recon_ != nullptr) {
        showSparseReconstruction(sparse_recon_);
    }

    std::cout << "zNear: " << camera()->zNear() << ", zFar: " << camera()->zFar() << std::endl;
}

void ViewerWidget::setUseDroneCamera(bool use_drone_camera) {
    if (use_drone_camera) {
        const PinholeCamera& pinhole_camera = sparse_recon_->getCameras().cbegin()->second;
        double fy = pinhole_camera.getFocalLengthY();
        double v_fov = 2 * std::atan(pinhole_camera.height() / (2 * fy));
        camera()->setFieldOfView(v_fov);
        aspect_ratio_ = pinhole_camera.width() / static_cast<double>(pinhole_camera.height());
        updateGeometry();
        std::cout << "Setting camera FOV to " << (v_fov * 180 / M_PI) << " degrees" << std::endl;
        std::cout << "Resized window to " << width() << " x " << height() << std::endl;
    }
    else {
        double v_fov = M_PI / 4;
        camera()->setFieldOfView(v_fov);
        std::cout << "Setting camera FOV to " << (v_fov * 180 / M_PI) << std::endl;
    }
}

int ViewerWidget::heightForWidth(int w) const {
    if (aspect_ratio_ <= 0) {
        return -1;
    }
    return static_cast<int>(w / aspect_ratio_);
}

QSize ViewerWidget::sizeHint() const {
    if (aspect_ratio_ <= 0) {
        return QSize();
    }
    return QSize(width(), width() / aspect_ratio_);
}

void ViewerWidget::setImagePoseIndex(ImageId image_id) {
    const Image& image = sparse_recon_->getImages().at(image_id);
    setCameraPose(image.pose);
}

void ViewerWidget::showOctree(const octomap::OcTree* octree) {
    octree_ = octree;
    if (!initialized_) {
        return;
    }

    // update viewer stat
    double minX, minY, minZ, maxX, maxY, maxZ;
    minX = minY = minZ = -10; // min bbx for drawing
    maxX = maxY = maxZ = 10;  // max bbx for drawing
    double sizeX, sizeY, sizeZ;
    sizeX = sizeY = sizeZ = 0.;
    size_t memoryUsage = 0;
    size_t num_nodes = 0;

    // get map bbx
    double lminX, lminY, lminZ, lmaxX, lmaxY, lmaxZ;
    octree_->getMetricMin(lminX, lminY, lminZ);
    octree_->getMetricMax(lmaxX, lmaxY, lmaxZ);
    // transform to world coords using map origin
    octomap::point3d pmin(lminX, lminY, lminZ);
    octomap::point3d pmax(lmaxX, lmaxY, lmaxZ);
    lminX = pmin.x(); lminY = pmin.y(); lminZ = pmin.z();
    lmaxX = pmax.x(); lmaxY = pmax.y(); lmaxZ = pmax.z();
    // update global bbx
    if (lminX < minX) minX = lminX;
    if (lminY < minY) minY = lminY;
    if (lminZ < minZ) minZ = lminZ;
    if (lmaxX > maxX) maxX = lmaxX;
    if (lmaxY > maxY) maxY = lmaxY;
    if (lmaxZ > maxZ) maxZ = lmaxZ;
    double lsizeX, lsizeY, lsizeZ;
    // update map stats
    octree_->getMetricSize(lsizeX, lsizeY, lsizeZ);
    if (lsizeX > sizeX) sizeX = lsizeX;
    if (lsizeY > sizeY) sizeY = lsizeY;
    if (lsizeZ > sizeZ) sizeZ = lsizeZ;
    memoryUsage += octree_->memoryUsage();
    num_nodes += octree_->size();

    octree_drawer_.enableHeightColorMode();
    refreshTree();

    setSceneBoundingBox(qglviewer::Vec(minX, minY, minZ), qglviewer::Vec(maxX, maxY, maxZ));
}

void ViewerWidget::showSparseReconstruction(const SparseReconstruction* sparse_recon)
{
    sparse_recon_ = sparse_recon;
    if (!initialized_) {
        return;
    }

    sparce_recon_drawer_.setSparseReconstruction(sparse_recon_);

    // Fill camera poses dropbox in settings panel
    std::vector<std::pair<std::string, ImageId>> pose_entries;
    for (const auto& image_entry : sparse_recon_->getImages()) {
        pose_entries.push_back(std::make_pair(image_entry.second.name, image_entry.first));
    }
    settings_panel_->initializeImagePoses(pose_entries);

    update();
}

void ViewerWidget::refreshTree()
{
    if (octree_ != nullptr) {
        octree_drawer_.setOctree(octree_);
    }
    update();
}

void ViewerWidget::resetView()
{
    this->camera()->setOrientation((float) -M_PI / 2.0f, (float) M_PI / 2.0f);
    this->showEntireScene();
    updateGL();
}

qglviewer::Vec ViewerWidget::eigenToQglviewer(const Eigen::Vector3d& eig_vec) const {
    return qglviewer::Vec(eig_vec(0), eig_vec(1), eig_vec(2));
}

Eigen::Vector3d ViewerWidget::qglviewerToEigen(const qglviewer::Vec& qgl_vec) const {
    Eigen::Vector3d eig_vec;
    eig_vec << qgl_vec.x, qgl_vec.y, qgl_vec.z;
    return eig_vec;
}

qglviewer::Quaternion ViewerWidget::eigenToQglviewer(const Eigen::Quaterniond& eig_quat) const {
    qglviewer::Quaternion qgl_quat(eig_quat.x(), eig_quat.y(), eig_quat.z(), eig_quat.w());
    return qgl_quat;
}

Eigen::Quaterniond ViewerWidget::qglviewerToEigen(const qglviewer::Quaternion& qgl_quat) const {
    Eigen::Quaterniond eig_quat;
    eig_quat.x() = qgl_quat[0];
    eig_quat.y() = qgl_quat[1];
    eig_quat.z() = qgl_quat[2];
    eig_quat.w() = qgl_quat[3];
    return eig_quat;
}

// Return camera pose (transformation from world to camera coordinate system)
Pose ViewerWidget::getCameraPose() const {
    Pose inv_pose;
    inv_pose.translation() = qglviewerToEigen(camera()->position());
    // Convert to OpenGL camera coordinate system (x is right, y is up, z is back)
    Eigen::AngleAxisd rotate_x_pi = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    inv_pose.quaternion() = qglviewerToEigen(camera()->orientation()) * rotate_x_pi;
    return inv_pose.inverse();
//    qglviewer::Vec translation = this->camera()->frame()->translation();
//    qglviewer::Quaternion quaternion = this->camera()->frame()->rotation();
//    quaternion.invert();
//    translation = - (quaternion * translation);
//    Eigen::Matrix4d camera_pose = Eigen::Matrix4d::Identity();
//    for (size_t i = 0; i < 4; ++i) {
//        for (size_t j = 0; j < 4; ++j) {
//            camera_pose(i, j) = quaternion.matrix()[j * 4 + i];
//        }
//    }
//    for (size_t i = 0; i < 3; ++i) {
//        camera_pose(i, 3) = translation.v_[i];
//    }
//    return camera_pose;
}

// Set camera pose (transformation from world to camera coordinate system)
void ViewerWidget::setCameraPose(const Pose& pose) {
    Pose inv_pose = pose.inverse();
    camera()->setPosition(eigenToQglviewer(inv_pose.translation()));
    // Convert to OpenGL camera coordinate system (x is right, y is up, z is back)
    Eigen::AngleAxisd rotate_x_pi = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    camera()->setOrientation(eigenToQglviewer(inv_pose.quaternion() * rotate_x_pi));
}

void ViewerWidget::setOccupancyThreshold(double occupancy_threshold)
{
//    std::cout << "Setting occupancy threshold to " << occupancy_threshold << std::endl;
    octree_drawer_.setOccupancyThreshold(occupancy_threshold);
    update();
}


void ViewerWidget::setDrawFreeVoxels(bool draw_free_voxels)
{
    octree_drawer_.setDrawFreeVoxels(draw_free_voxels);
}

void ViewerWidget::setDisplayAxes(bool display_axes)
{
    octree_drawer_.setDisplayAxes(display_axes);
}

void ViewerWidget::setVoxelAlpha(double voxel_alpha)
{
//    std::cout << "Setting voxel alpha to " << voxel_alpha << std::endl;
    octree_drawer_.setAlphaOccupied(voxel_alpha);
    update();
}

void ViewerWidget::setRenderTreeDepth(int render_tree_depth)
{
//    std::cout << "Setting render tree depth to " << render_tree_depth << std::endl;
    octree_drawer_.setRenderTreeDepth(render_tree_depth);
}

void ViewerWidget::setDrawSingleBin(bool draw_single_bin)
{
    octree_drawer_.setDrawSingleBin(draw_single_bin);
    update();
}

void ViewerWidget::setDrawOctree(bool draw_octree)
{
    draw_octree_ = draw_octree;
}

void ViewerWidget::setDrawCameras(bool draw_cameras)
{
    sparce_recon_drawer_.setDrawCameras(draw_cameras);
}

void ViewerWidget::setDrawSparsePoints(bool draw_sparse_points)
{
    sparce_recon_drawer_.setDrawSparsePoints(draw_sparse_points);
}

void ViewerWidget::setSceneBoundingBox(const qglviewer::Vec& min, const qglviewer::Vec& max)
{
    qglviewer::Vec min_vec(-50, -50, -20);
    qglviewer::Vec max_vec(50, 50, 50);
    QGLViewer::setSceneBoundingBox(min_vec, max_vec);
}

void ViewerWidget::draw()
{
    // debugging: draw light in scene
    //drawLight(GL_LIGHT0);

//    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_VERTEX_PROGRAM_POINT_SIZE);

    glEnable(GL_LIGHTING);
//    glCullFace(GL_BACK);
    glDisable(GL_CULL_FACE);

//    camera()->fitBoundingBox(qglviewer::Vec(-10, -10, -10), qglviewer::Vec(10, 10, 10));
//    camera()->setSceneCenter(qglviewer::Vec(0, 0, 0));
    QMatrix4x4 pmv_matrix;
    camera()->getModelViewProjectionMatrix(pmv_matrix.data());

    // draw drawable objects:
    if (draw_octree_) {
        octree_drawer_.draw();
    }
    sparce_recon_drawer_.draw(pmv_matrix, width(), height());
}

void ViewerWidget::drawWithNames()
{
}

void ViewerWidget::postDraw()
{
    // Reset model view matrix to world coordinates origin
    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    camera()->loadModelViewMatrix();
    // TODO restore model loadProjectionMatrixStereo

    // Save OpenGL state
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glDisable(GL_COLOR_MATERIAL);
    qglColor(foregroundColor());

    if (gridIsDrawn()){
        glLineWidth(1.0);
        drawGrid(5.0, 10);
    }
    if (axisIsDrawn()){
        glLineWidth(2.0);
        drawAxis(1.0);
    }

    // Restore GL state
    glPopAttrib();
    glPopMatrix();

    bool drawAxis = axisIsDrawn();
    bool drawGrid = gridIsDrawn();
    setAxisIsDrawn(false);
    setGridIsDrawn(false);
    QGLViewer::postDraw();

    setAxisIsDrawn(drawAxis);
    setGridIsDrawn(drawGrid);
}

void ViewerWidget::postSelection(const QPoint&)
{
}

void ViewerWidget::wheelEvent(QWheelEvent* event)
{
    if (event->modifiers() & Qt::ControlModifier) {
        sparce_recon_drawer_.changePointSize(event->delta());
        event->accept();
        updateGL();
    } else if (event->modifiers() & Qt::AltModifier) {
        sparce_recon_drawer_.changeCameraSize(event->delta());
        event->accept();
        updateGL();
    } else if (event->modifiers() & Qt::ShiftModifier) {
//      ChangeNearPlane(event->delta());
        QGLViewer::wheelEvent(event);
    } else {
//      ChangeFocusDistance(event->delta());
        QGLViewer::wheelEvent(event);
    }
}
