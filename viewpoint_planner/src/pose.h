//==================================================
// pose.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 9, 2016
//==================================================
#pragma once

#include <ait/eigen_utils.h>

// A pose gives the transformation from world to image coordinate system
struct Pose
{
    Pose()
    : translation_(0, 0, 0), quaternion_(1, 0, 0, 0) {}

    Pose(const Eigen::Vector3d& translation, const Eigen::Quaterniond& quaternion)
    : translation_(translation), quaternion_(quaternion) {}

    Pose(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation)
    : translation_(translation), quaternion_(rotation) {}

    Pose(const Eigen::Matrix4d& matrix)
    : translation_(matrix.col(3).topRows<3>()), quaternion_(matrix.topLeftCorner<3, 3>()) {}

    Eigen::Vector3d& translation() {
        return translation_;
    }

    const Eigen::Vector3d& translation() const {
        return translation_;
    }

    Eigen::Quaterniond& quaternion() {
        return quaternion_;
    }

    const Eigen::Quaterniond& quaternion() const {
        return quaternion_;
    }

    Eigen::Vector3d getWorldPosition() const {
        return - (quaternion_.inverse() * translation_);
    }

    Pose inverse() const {
        Pose inv_pose;
        inv_pose.quaternion() = quaternion_.inverse();
        inv_pose.translation() = - (inv_pose.quaternion() * translation_);
        return inv_pose;
    }

    Eigen::Matrix3d rotation() const {
        return quaternion_.toRotationMatrix();
    }

    Eigen::Matrix3x4d getTransformationWorldToImage() const {
        Eigen::Matrix3x4d transformation;
        transformation.leftCols<3>() = quaternion_.toRotationMatrix();
        transformation.rightCols<1>() = translation_;
        return transformation;
    }

    Eigen::Matrix3x4d getTransformationImageToWorld() const {
        Eigen::Matrix3x4d transformation_world_to_camera = getTransformationWorldToImage();
        Eigen::Matrix3x4d transformation;
        transformation.leftCols<3>() = transformation_world_to_camera.leftCols<3>().transpose();
        transformation.rightCols<1>() = - transformation.leftCols<3>() * transformation_world_to_camera.rightCols<1>();
        return transformation;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
    Eigen::Vector3d translation_;
    Eigen::Quaterniond quaternion_;
};
