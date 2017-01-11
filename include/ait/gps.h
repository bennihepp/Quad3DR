//==================================================
// gps.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 10, 2017
//==================================================
#pragma once

#include <iostream>
#include <iomanip>
#include <ait/math.h>
#include <ait/eigen.h>

namespace ait {

template <typename FloatT>
class GpsCoordinate {
public:
  using FloatType = FloatT;

  GpsCoordinate()
  : lat_(0), long_(0) {}

  GpsCoordinate(const FloatType latitude, const FloatType longitude)
  : lat_(latitude), long_(longitude) {}

  FloatType latitude() const {
    return lat_;
  }

  FloatType longitude() const {
    return long_;
  }

private:
  FloatType lat_;
  FloatType long_;
};

template <typename FloatT>
std::ostream& operator<<(std::ostream& out, const GpsCoordinate<FloatT>& gps) {
  auto prec = out.precision();
  out << std::fixed << std::setprecision(15);
  out << "(" << gps.latitude() << ", " << gps.longitude() << ")";
  out.precision(prec);
  out.unsetf(std::ios_base::floatfield);
  return out;
}

template <typename FloatT>
class GpsCoordinateWithAltitude : public GpsCoordinate<FloatT> {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType)
  using BaseType = GpsCoordinate<FloatType>;

  GpsCoordinateWithAltitude()
  : altitude_(0) {}

  GpsCoordinateWithAltitude(const FloatType latitude, const FloatType longitude, const FloatType altitude)
  : BaseType(latitude, longitude), altitude_(altitude) {}

  GpsCoordinateWithAltitude(const Vector3& vec)
  : BaseType(vec(0), vec(1)), altitude_(vec(2)) {}

  FloatType altitude() const {
    return altitude_;
  }

private:
  FloatType lat_;
  FloatType long_;
  FloatType altitude_;
};

template <typename FloatT>
std::ostream& operator<<(std::ostream& out, const GpsCoordinateWithAltitude<FloatT>& gps) {
  auto prec = out.precision();
  out << std::fixed << std::setprecision(15);
  out << "(" << gps.latitude() << ", " << gps.longitude();
  out.precision(prec);
  out.unsetf(std::ios_base::floatfield);
  out << ", " << gps.altitude() << ")";
  return out;
}

template <typename FloatT>
class GpsConverter {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType)
  using GpsCoordinateType = GpsCoordinateWithAltitude<FloatType>;

  static GpsConverter createWGS84(const GpsCoordinateType& reference) {
    const FloatType a = (FloatType)6378137.;
    const FloatType b = (FloatType)6356752.3142;
    return GpsConverter(reference, a, b);
  }

  GpsConverter(const GpsCoordinateType& gps_reference, const FloatType a, const FloatType b)
  : a_square_(a * a), b_square_(b * b), gps_reference_(gps_reference),
    ecef_reference_(convertGpsToEcef(gps_reference)) {}

  Vector3 convertGpsToEcef(const GpsCoordinateType& gps) const {
    const FloatType latitude = ait::degreeToRadians(gps.latitude());
    const FloatType longitude = ait::degreeToRadians(gps.longitude());
    const FloatType altitude = gps.altitude();
    const FloatType cos_latitude = std::cos(latitude);
    const FloatType sin_latitude = std::sin(latitude);
    const FloatType cos_longitude = std::cos(longitude);
    const FloatType sin_longitude = std::sin(longitude);
    const FloatType N = a_square_ / std::sqrt(a_square_ * cos_latitude * cos_latitude + b_square_ * sin_latitude * sin_latitude);
    const FloatType x = (N + altitude) * cos_latitude * cos_longitude;
    const FloatType y = (N + altitude) * cos_latitude * sin_longitude;
    const FloatType z = (b_square_ / a_square_ * N + altitude) * sin_latitude;
    return Vector3(x, y, z);
  }

  /// Convert ECEF to ENU using the previously set Gps reference point
  Vector3 convertGpsToEnu(const GpsCoordinateType& gps) const {
    const FloatType latitude = ait::degreeToRadians(gps.latitude());
    const FloatType longitude = ait::degreeToRadians(gps.longitude());
    const FloatType altitude = gps.altitude();
    const FloatType cos_latitude = std::cos(latitude);
    const FloatType sin_latitude = std::sin(latitude);
    const FloatType cos_longitude = std::cos(longitude);
    const FloatType sin_longitude = std::sin(longitude);
    const Vector3 ecef = convertGpsToEcef(gps);
    const Vector3 XYZ = ecef - ecef_reference_;
    const FloatType x = -sin_longitude * XYZ(0) + cos_longitude * XYZ(1);
    const FloatType y = -sin_latitude * cos_longitude * XYZ(0) - sin_latitude * sin_longitude * XYZ(1) + cos_latitude * XYZ(2);
    const FloatType z = cos_latitude * cos_longitude * XYZ(0) + cos_latitude * sin_longitude * XYZ(1) + sin_latitude * XYZ(2);
    return Vector3(x, y, z);
  }

private:
  const FloatType a_square_;
  const FloatType b_square_;
  const GpsCoordinateType gps_reference_;
  const Vector3 ecef_reference_;
};

}
