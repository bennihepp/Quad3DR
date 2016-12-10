//==================================================
// qt_utils.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Dec 6, 2016
//==================================================
#pragma once

#include <iostream>

#include <QVector3D>
#include <QVector4D>
#include <QMatrix3x3>
#include <QMatrix4x4>

template <typename Char>
std::basic_ostream<Char>& operator<<(std::basic_ostream<Char>& out, const QVector3D& vec)
{
    out << "(" << vec.x() << ", " << vec.y() << ", " << vec.z() << ")";
    return out;
}

template <typename Char>
std::basic_ostream<Char>& operator<<(std::basic_ostream<Char>& out, const QVector4D& vec)
{
    out << "(" << vec.x() << ", " << vec.y() << ", " << vec.z() << ", " << vec.w() << ")";
    return out;
}

template <typename Char>
std::basic_ostream<Char>& operator<<(std::basic_ostream<Char>& out, const QMatrix3x3& mat)
{
    out << "( " << mat(0, 0) << ", " << mat(0, 1) << ", " << mat(0, 2) << std::endl;
    out << "  " << mat(1, 0) << ", " << mat(1, 1) << ", " << mat(1, 2) << std::endl;
    out << "  " << mat(2, 0) << ", " << mat(2, 1) << ", " << mat(2, 2) << " )";
    return out;
}

template <typename Char>
std::basic_ostream<Char>& operator<<(std::basic_ostream<Char>& out, const QMatrix4x4& mat)
{
    out << "( " << mat(0, 0) << ", " << mat(0, 1) << ", " << mat(0, 2) << ", " << mat(0, 3) << std::endl;
    out << "  " << mat(1, 0) << ", " << mat(1, 1) << ", " << mat(1, 2) << ", " << mat(1, 3) << std::endl;
    out << "  " << mat(2, 0) << ", " << mat(2, 1) << ", " << mat(2, 2) << ", " << mat(2, 3) << std::endl;
    out << "  " << mat(3, 0) << ", " << mat(3, 1) << ", " << mat(3, 2) << ", " << mat(3, 3) << " )";
    return out;
}
