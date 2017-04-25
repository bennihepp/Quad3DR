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
#include <bh/math/utilities.h>
#include <bh/eigen.h>

namespace bh {

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

  FloatType latitudeRadians() const {
    return bh::degreeToRadians(lat_);
  }

  FloatType longitudeRadians() const {
    return bh::degreeToRadians(long_);
  }

  template <typename T>
  GpsCoordinate<T> cast() const {
    const GpsCoordinate<T> other(this->latitude(), this->longitude());
    return other;
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

  template <typename T>
  GpsCoordinateWithAltitude<T> cast() const {
    const GpsCoordinateWithAltitude<T> other(this->latitude(), this->longitude(), this->altitude());
    return other;
  }

private:
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
      : a_(a), a_square_(a * a), b_square_(b * b),
        e_square_(1 - b_square_ / a_square_),
        e_prime_square_((a_square_ - b_square_) / b_square_),
        gps_reference_(gps_reference),
        ecef_reference_(convertGpsToEcef(gps_reference)),
        ref_cos_latitude_(std::cos(bh::degreeToRadians(gps_reference_.latitude()))),
        ref_sin_latitude_(std::sin(bh::degreeToRadians(gps_reference_.latitude()))),
        ref_cos_longitude_(std::cos(bh::degreeToRadians(gps_reference_.longitude()))),
        ref_sin_longitude_(std::sin(bh::degreeToRadians(gps_reference_.longitude()))),
        ref_N_(a_square_ / std::sqrt(a_square_ * ref_cos_latitude_ * ref_cos_latitude_ + b_square_ * ref_sin_latitude_ * ref_sin_latitude_)) {}

  GpsConverter(const GpsConverter& other) = default;

  GpsConverter(GpsConverter&& other) = default;

  Vector3 convertGpsToEcef(const GpsCoordinateType& gps) const {
    const FloatType latitude = bh::degreeToRadians(gps.latitude());
    const FloatType longitude = bh::degreeToRadians(gps.longitude());
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

  GpsCoordinateType convertEcefToGps(const Vector3& ecef) const {
    const FloatType X = ecef(0);
    const FloatType Y = ecef(1);
    const FloatType Z = ecef(2);
    // Ferrari's solution using Zhu's method (https://en.wikipedia.org/wiki/Geographic_coordinate_conversion#The_application_of_Ferrari.27s_solution)
    const FloatType r = std::sqrt(X * X + Y * Y);
    const FloatType E_square = a_square_ - b_square_;
    const FloatType F = 54 * b_square_ * Z * Z;
    const FloatType G = r * r + (1 - e_square_) * Z * Z - e_square_ * E_square;
    const FloatType C = e_square_ * e_square_ * F * r * r / (G * G * G);
    const FloatType S = std::pow(1 + C + std::sqrt(C * C + 2 * C), 1 / (FloatType)3);
    const FloatType SP_tmp = (S + 1 / S + 1);
    const FloatType P = F / (3 * SP_tmp * SP_tmp * G * G);
    const FloatType Q = std::sqrt(1 + 2 * e_square_ * e_square_ * P);
    const FloatType r0 = -(P * e_square_ * r) / (1 + Q) + std::sqrt(
        a_square_ / 2 * (1 + 1 / Q) - P * (1 - e_square_) * Z * Z / (Q * (1 + Q)) - P * r * r / 2);
    const FloatType U = std::sqrt((r - e_square_ * r0) + Z * Z);
    const FloatType V_tmp = (r - e_square_ * r0);
    const FloatType V = std::sqrt(V_tmp * V_tmp + (1 - e_square_) * Z * Z);
    const FloatType Z0 = b_square_ * Z / (a_ * V);
    const FloatType h = U * (1 - b_square_ / (a_ * V));
    const FloatType latitude = std::atan((Z + e_prime_square_ * Z0) / (r));
    const FloatType longitude = std::atan2(Y, X);
    const FloatType cos_latitude = std::cos(latitude);
    const FloatType sin_latitude = std::sin(latitude);
    const FloatType cos_longitude = std::cos(longitude);
    const FloatType sin_longitude = std::sin(longitude);
    const FloatType N = a_square_ / std::sqrt(a_square_ * cos_latitude * cos_latitude + b_square_ * sin_latitude * sin_latitude);
    const FloatType altitude = Z / sin_latitude - (b_square_ / a_square_ * N);
    const FloatType latitude_degrees = bh::radiansToDegrees(latitude);
    const FloatType longitude_degrees = bh::radiansToDegrees(longitude);
    return GpsCoordinateType(latitude_degrees, longitude_degrees, altitude);
  }

  /// Convert ECEF to ENU using the previously set Gps reference point
  Vector3 convertEcefToEnu(const Vector3& ecef) const {
    const Vector3 XYZ = ecef - ecef_reference_;
    const FloatType x = -ref_sin_longitude_ * XYZ(0) + ref_cos_longitude_ * XYZ(1);
    const FloatType y = -ref_sin_latitude_ * ref_cos_longitude_ * XYZ(0) - ref_sin_latitude_ * ref_sin_longitude_ * XYZ(1) + ref_cos_latitude_ * XYZ(2);
    const FloatType z = ref_cos_latitude_ * ref_cos_longitude_ * XYZ(0) + ref_cos_latitude_ * ref_sin_longitude_ * XYZ(1) + ref_sin_latitude_ * XYZ(2);
    return Vector3(x, y, z);
  }

  /// Convert ENU to ECEF using the previously set Gps reference point
  Vector3 convertEnuToEcef(const Vector3& enu) const {
    const FloatType x = -ref_sin_longitude_ * enu(0) - ref_sin_latitude_ * ref_cos_longitude_ * enu(1) + ref_cos_latitude_ * ref_cos_longitude_ * enu(2);
    const FloatType y = ref_cos_longitude_ * enu(0) - ref_sin_latitude_ * ref_sin_longitude_ * enu(1) + ref_cos_latitude_ * ref_sin_longitude_ * enu(2);
    const FloatType z = ref_cos_latitude_ * enu(1) + ref_sin_latitude_ * enu(2);
    return Vector3(x, y, z) + ecef_reference_;
  }

  /// Convert GPS to ENU using the previously set Gps reference point
  Vector3 convertGpsToEnu(const GpsCoordinateType& gps) const {
    const Vector3 ecef = convertGpsToEcef(gps);
    return convertEcefToEnu(ecef);
  }

  GpsCoordinateType convertEnuToGps(const Vector3& enu) const {
    const Vector3 ecef = convertEnuToEcef(enu);
    return convertEcefToGps(ecef);
  }

private:
  const FloatType a_;
  const FloatType a_square_;
  const FloatType b_square_;
  const FloatType e_square_;
  const FloatType e_prime_square_;
  const GpsCoordinateType gps_reference_;
  const Vector3 ecef_reference_;
  const FloatType ref_cos_latitude_;
  const FloatType ref_sin_latitude_;
  const FloatType ref_cos_longitude_;
  const FloatType ref_sin_longitude_;
  const FloatType ref_N_;
};

}
