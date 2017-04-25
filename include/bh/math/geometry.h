//==================================================
// geometry.h
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Nov 8, 2017
//==================================================

#pragma once

#include <cmath>
#include <boost/serialization/access.hpp>
#include <boost/serialization/split_member.hpp>
#include "../common.h"
#include "../../ait/eigen.h"

namespace bh {

#if __GNUC__ && !__CUDACC__
  #pragma GCC push_options
  #pragma GCC optimize ("fast-math")
#endif

// General geometry functions
// Project vector v onto v_base
template <typename Derived1, typename Derived2>
typename Derived1::PlainObject projectVector(const Derived1& v, const Derived2& v_base) {
  typename Derived1::PlainObject v_proj = v_base;
  v_proj.normalize();
  v_proj *= v_proj.dot(v);
  return v_proj;
}

//template <typename T>
//struct Ray {
//  using FloatType = T;
//  USE_FIXED_EIGEN_TYPES(FloatType)
//
//  Ray(const Vector3& origin, const Vector3& direction)
//  : origin_(origin), direction_(direction.normalized()) {}
//
//  const Vector3& origin() const {
//    return origin_;
//  }
//
//  const Vector3& direction() const {
//    return direction_;
//  }
//
//  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
//
//private:
//  Vector3 origin_;
//  Vector3 direction_;
//};

template <typename FloatT>
struct Ray {
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType)

  Ray()
      : origin(Vector3::Zero()), direction(Vector3::UnitX()) {}

  Ray(const Vector3& origin, const Vector3& direction)
      : origin(origin), direction(direction.normalized()) {}

  void setOrigin(const Vector3& origin) {
    this->origin = origin;
  }

  void setDirection(const Vector3& direction) {
    this->direction = direction.normalized();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector3 origin;
  Vector3 direction;
};

template <typename FloatT>
struct RayData : public Ray<FloatT> {
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType)
  using RayType = Ray<FloatT>;

  RayData()
      : RayType(Vector3::Zero(), Vector3::UnitX()),
        inv_direction(RayType::direction.cwiseInverse()) {}

  RayData(const Vector3& origin, const Vector3& direction)
      : RayType(origin, direction),
        inv_direction(RayType::direction.cwiseInverse()) {}

  RayData(const RayType& ray)
      : RayType(ray),
        inv_direction(RayType::direction.cwiseInverse()) {}

  void setDirection(const Vector3& direction) {
    RayType::setDirection(direction);
    this->inv_direction = this->direction.cwiseInverse();
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Vector3 inv_direction;
  FloatType min_range_sq;
  FloatType max_range_sq;
};

template <typename FloatT>
class RayIntersection {
public:
  RayIntersection()
      : does_intersect_(false) {}

  RayIntersection(const bool does_intersect, const FloatT ray_t)
      : does_intersect_(does_intersect), ray_t_(ray_t) {}

  bool doesIntersect() const { return does_intersect_; }

  FloatT rayT() const { return ray_t_; }

private:
  bool does_intersect_;
  FloatT ray_t_;
};

template <typename FloatT = float>
class BoundingBox3D {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatT)
  using RayType = Ray<FloatT>;
  using RayDataType = RayData<FloatT>;
  using RayIntersectionType = RayIntersection<FloatT>;

  static BoundingBox3D createFromCenterAndExtent(const Vector3& center, const Vector3& extent) {
    const Vector3 min = center - extent / 2;
    const Vector3 max = center + extent / 2;
    return BoundingBox3D(min, max);
  }

  template <typename PointIterator>
  static BoundingBox3D createFromPoints(PointIterator first, PointIterator last) {
    BoundingBox3D bbox;
    for (PointIterator it = first; it != last; ++it) {
      bbox.include(*it);
    }
    return bbox;
  }

  BoundingBox3D() {
    min_ << std::numeric_limits<FloatT>::max(), std::numeric_limits<FloatT>::max(), std::numeric_limits<FloatT>::max();
    max_ = -min_;
  }

  BoundingBox3D(const Vector3& center, FloatType size)
  : min_(center.array() - size / 2), max_(center.array() + size / 2) {}

  BoundingBox3D(const Vector3& min, const Vector3& max)
  : min_(min), max_(max) {}

  ~BoundingBox3D() {}

  bool operator==(const BoundingBox3D& other) const {
    return min_ == other.min_ && max_ == other.max_;
  }

  bool isValid() const {
    return (min_.array() <= max_.array()).all();
  }

  bool isEmpty() const {
    return (min_.array() >= max_.array()).all();
  }

  BoundingBox3D operator+(const Vector3& offset) const {
    return BoundingBox3D(getMinimum() + offset, getMaximum() + offset);
  }

  const Vector3& getMinimum() const {
    return min_;
  }

  const FloatType getMinimum(std::size_t index) const {
    return min_(index);
  }

  const Vector3& getMaximum() const {
    return max_;
  }

  const FloatType getMaximum(std::size_t index) const {
    return max_(index);
  }

  const Vector3 getExtent() const {
    return max_ - min_;
  }

  FloatType getExtent(std::size_t index) const {
    return (max_ - min_)(index);
  }

  const FloatType getMaxExtent() const {
    return (max_ - min_).maxCoeff();
  }

  const FloatType getMaxExtent(std::size_t* index) const {
    return (max_ - min_).maxCoeff(index);
  }

  const Vector3 getCenter() const {
    return (min_ + max_) / 2;
  }

  const FloatType getCenter(std::size_t index) const {
    return (min_(index) + max_(index)) / 2;
  }

  FloatType getVolume() const {
    return getExtent().array().prod();
  }

  bool isOutside(const Vector3& point) const {
    return (point.array() < min_.array()).any()
        || (point.array() > max_.array()).any();
  }

  bool isInside(const Vector3& point) const {
    return (point.array() >= min_.array()).all()
        && (point.array() <= max_.array()).all();
  }

  bool isOutsideOf(const BoundingBox3D& bbox) const {
    return (max_.array() < bbox.min_.array()).any()
        || (min_.array() > bbox.max_.array()).any();
  }

  bool isInsideOf(const BoundingBox3D& bbox) const {
    return (max_.array() >= bbox.min_.array()).all()
        && (min_.array() <= bbox.max_.array()).all();
  }

  /// Return squared distance to closest point on the outside (if point is inside distance is 0)
  FloatType squaredDistanceTo(const Vector3& point) const {
    FloatType dist_sq = 0;
    for (std::size_t i = 0; i < 3; ++i) {
      bool outside = point(i) < getMinimum(i) || point(i) > getMaximum(i);
      if (outside) {
        FloatType d = std::min(std::abs(point(i) - getMinimum(i)), std::abs(point(i) - getMaximum(i)));
        dist_sq += d * d;
      }
    }
    return dist_sq;
  }

  /// Return distance to closest point on the outside (if point is inside distance is 0)
  FloatType distanceTo(const Vector3& point) const {
    return std::sqrt(squaredDistanceTo(point));
  }

  bool contains(const BoundingBox3D& other) const {
    return other.isInsideOf(*this);
  }

  bool intersects(const BoundingBox3D& other) const {
    if ((max_.array() < other.min_.array()).any()) {
      return false;
    }
    if ((other.max_.array() < min_.array()).any()) {
      return false;
    }
    return true;
  }

  /// Intersect ray with bounding box.
  ///
  /// @param t_min: Minimum ray equation coefficient, >= 0.
  /// @param t_max: Maximum ray equation coefficient, > t_min.
  /// @return: A ray intersection object with the intersection result and the ray coefficient of the intersection point.
  RayIntersectionType intersect(const RayDataType& ray,
                                const FloatT t_min = 0,
                                const FloatT t_max = std::numeric_limits<FloatT>::max()) const {
    // Faster version without explicit check for directions parallel to an axis
    FloatT t_lower = t_min;
    FloatT t_upper = t_max;
    const size_t kDimensions = 3;
    for (size_t i = 0; i < kDimensions; ++i) {
      const FloatT t0 = (min_(i) - ray.origin(i)) * ray.inv_direction(i);
      const FloatT t1 = (max_(i) - ray.origin(i)) * ray.inv_direction(i);
      t_lower = std::max(t_lower, std::min(t0, t1));
      t_upper = std::min(t_upper, std::max(t0, t1));
    }
    if (t_upper >= t_lower) {
      return RayIntersectionType(true, t_lower);
    }
    else {
      return RayIntersectionType(false, 0);
    }
  }

  bool intersects(const RayType& ray, Vector3* intersection = nullptr) const {
    RayDataType ray_data(ray);
    ray_data.inv_direction = ray.direction.cwiseInverse();
    return intersects(ray_data, intersection);
  }

  bool intersects(const RayDataType& ray_data, Vector3* intersection = nullptr) const {
    float t_min = -std::numeric_limits<FloatType>::max();
    float t_max = std::numeric_limits<FloatType>::max();

  //  for (std::size_t i = 0; i < 3; ++i) {
  //    if (ray_data.direction(i) != 0) {
  //      float t0 = (min_(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
  //      float t1 = (max_(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
  //      t_min = std::max(t_min, std::min(t0, t1));
  //      t_max = std::min(t_max, std::max(t0, t1));
  //    }
  //    else {
  //      if (ray_data.origin(i) <= min_(i) || ray_data.origin(i) >= max_(i)) {
  //        return false;
  //      }
  //    }
  //  }

    // Faster version without explicit check for directions parallel to an axis
    for (std::size_t i = 0; i < 3; ++i) {
      float t0 = (min_(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
      float t1 = (max_(i) - ray_data.origin(i)) * ray_data.inv_direction(i);
      t_min = std::max(t_min, std::min(t0, t1));
      t_max = std::min(t_max, std::max(t0, t1));
    }

    bool intersect = t_max > std::max(t_min, FloatType(0));
    if (intersect && intersection != nullptr) {
      t_min = std::max(t_min, FloatType(0));
      *intersection = ray_data.origin + ray_data.direction * t_min;
    }
    return intersect;
  }

  BoundingBox3D operator*(FloatType scale) const {
    return BoundingBox3D::createFromCenterAndExtent(getCenter(), scale * getExtent());
  }

  static BoundingBox3D getUnion(const BoundingBox3D& bbox_a, const BoundingBox3D& bbox_b) {
    Vector3 min = bbox_a.min_.cwiseMin(bbox_b.min_);
    Vector3 max = bbox_a.max_.cwiseMax(bbox_b.max_);
    return BoundingBox3D(min, max);
  }

  void include(const BoundingBox3D& other) {
    min_ = min_.cwiseMin(other.min_);
    max_ = max_.cwiseMax(other.max_);
  }

  void include(const Vector3& point) {
    min_ = min_.cwiseMin(point);
    max_ = max_.cwiseMax(point);
  }

  void constrainTo(const BoundingBox3D& bbox) {
    min_ = min_.cwiseMax(bbox.min_);
    max_ = max_.cwiseMin(bbox.max_);
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Boost serialization
  friend class boost::serialization::access;

  template <typename Archive>
  void load(Archive& ar, const unsigned int version) {
    ar & min_;
    ar & max_;
  }

  template <typename Archive>
  void save(Archive& ar, const unsigned int version) const {
    ar & min_;
    ar & max_;
  }

  BOOST_SERIALIZATION_SPLIT_MEMBER()

  Vector3 min_;
  Vector3 max_;
};

template <typename FloatType>
BoundingBox3D<FloatType> operator*(FloatType scale, const BoundingBox3D<FloatType>& bbox) {
  return BoundingBox3D<FloatType>::createFromCenterAndExtent(bbox.getCenter(), scale * bbox.getExtent());
}

using BoundingBox3Df = BoundingBox3D<float>;
using BoundingBox3Dd = BoundingBox3D<double>;

template <typename FloatType, typename CharType>
std::basic_ostream<CharType>& operator<<(std::basic_ostream<CharType>& out, const BoundingBox3D<FloatType>& bbox) {
  out << "(" << bbox.getMinimum(0) << ", " << bbox.getMinimum(1) << ", " << bbox.getMinimum(2) << ") -> ";
  out << "(" << bbox.getMaximum(0) << ", " << bbox.getMaximum(1) << ", " << bbox.getMaximum(2) << ")";
  return out;
}

template <typename FloatT>
class Triangle {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using RayIntersectionType = RayIntersection<FloatT>;

  Triangle()
  : v1_(Vector3::Zero()), v2_(Vector3::Zero()), v3_(Vector3::Zero()) {}

  Triangle(const Vector3& v1, const Vector3& v2, const Vector3& v3)
  : v1_(v1), v2_(v2), v3_(v3) {}

  Triangle(const Triangle& other)
  : v1_(other.v1_), v2_(other.v2_), v3_(other.v3_) {}

  Triangle(Triangle&& other)
  : v1_(std::move(other.v1_)), v2_(std::move(other.v2_)), v3_(std::move(other.v3_)) {}

  void operator=(const Triangle& other) {
    v1_ = other.v1_;
    v2_ = other.v2_;
    v3_ = other.v3_;
  }

  void operator=(Triangle&& other) {
    v1_ = std::move(other.v1_);
    v2_ = std::move(other.v2_);
    v3_ = std::move(other.v3_);
  }

  const Vector3& v1() const {
    return v1_;
  }

  const Vector3& v2() const {
    return v2_;
  }

  const Vector3& v3() const {
    return v3_;
  }

  Vector3& v1() {
    return v1_;
  }

  Vector3& v2() {
    return v2_;
  }

  Vector3& v3() {
    return v3_;
  }

  const Vector3& getVertex(const size_t i) const {
    return v_[i];
  }

  Vector3& getVertex(const size_t i) {
    return v_[i];
  }

  Vector3 getNormal(const bool normalized = true) const {
    Vector3 normal = (v2_ - v1_).cross(v3_ - v1_);
    if (normalized) {
      normal.normalize();
    }
    return normal;
  }

  Vector3 getBarycentricCoordinates(const Vector3& p) const {
    const Vector3 s0 = v2_ - v1_;
    const Vector3 s1 = v3_ - v1_;
    const Vector3 s2 = p - v1_;
    const FloatType d00 = s0.dot(s0);
    const FloatType d01 = s0.dot(s1);
    const FloatType d11 = s1.dot(s1);
    const FloatType d20 = s2.dot(s0);
    const FloatType d21 = s2.dot(s1);
    const FloatType denom = d00 * d11 - d01 * d01;
    const FloatType v = (d11 * d20 - d01 * d21) / denom;
    const FloatType w = (d00 * d21 - d01 * d20) / denom;
    const FloatType u = 1.0f - v - w;
    Vector3 bary(v, w, u);
    return bary;
  }

  BoundingBox3D<FloatT> boundingBox() const {
    const std::initializer_list<Vector3> points({ v1_, v2_, v3_ });
    const BoundingBox3D<FloatT> bbox = BoundingBox3D<FloatT>::createFromPoints(points.begin(), points.end());
    return bbox;
  }

  bool isInsideTriangle(const Vector3& p) const {
    const Vector3 bary = getBarycentricCoordinates(p);
    const bool inside = bary(0) >= 0 && bary(0) <= 1
        && bary(1) >= 0 && bary(1) <= 1
        && bary(2) >= 0 && bary(2) <= 1;
    return inside;
  }

  FloatType distanceToSurface(const Vector3& p) const {
    const Vector3 normal = getNormal();
    const FloatType distance = (p - v1_).dot(normal);
    return distance;
  }

  Vector3 projectPointOntoSurface(const Vector3& p) const {
    const Vector3 normal = getNormal();
    const FloatType dot_product = (p - v1_).dot(normal);
    const Vector3 proj_p = p - dot_product * normal;
    return proj_p;
  }

  bool doesPointProjectOntoTriangle(const Vector3& p) const {
    const Vector3 proj_p = projectPointOntoSurface(p);
    return isInsideTriangle(proj_p);
  }

  Triangle getCanonicalTriangle() const {
    const FloatType l1 = (v2_ - v1_).squaredNorm();
    const FloatType l2 = (v3_ - v2_).squaredNorm();
    const FloatType l3 = (v1_ - v3_).squaredNorm();
    Triangle can_tri;
    if (l1 >= l2) {
      if (l1 >= l3) {
        // l1 is longest
        can_tri = *this;
      }
      else {
        // l3 is longest
        can_tri.v1_ = v3_;
        can_tri.v2_ = v1_;
        can_tri.v3_ = v2_;
      }
    }
    else if (l2 >= l3) {
      // l2 is longest
      can_tri.v1_ = v2_;
      can_tri.v2_ = v3_;
      can_tri.v3_ = v1_;
    }
    else {
      // l3 is longest
      can_tri.v1_ = v3_;
      can_tri.v2_ = v1_;
      can_tri.v3_ = v2_;
    }
    BH_ASSERT((can_tri.v2_ - can_tri.v1_).squaredNorm() >= (can_tri.v3_ - can_tri.v2_).squaredNorm());
    BH_ASSERT((can_tri.v2_ - can_tri.v1_).squaredNorm() >= (can_tri.v1_ - can_tri.v3_).squaredNorm());
    return can_tri;
  }

  Vector3 getCenter() const {
    return (v1_ + v2_ + v3_) / 3;
  }

  FloatType computeTriangleAreaSquare() const {
    const Triangle can_tri = getCanonicalTriangle();
    const FloatType height_square = computeTriangleHeightSquare(can_tri.v2_ - can_tri.v1_, can_tri.v3_ - can_tri.v1_);
    const FloatType area_square = (v2_ - v1_).squaredNorm() * height_square / 4;
    return area_square;
  }

  std::array<Triangle, 2> splitTriangle() const {
    const Triangle can_tri = getCanonicalTriangle();
    // Now v2_ - v1_ is the longest edge
    const Vector3 v4 = (can_tri.v1_ + can_tri.v2_) / 2;
    Triangle tri1;
    tri1.v1_ = can_tri.v1_;
    tri1.v2_ = v4;
    tri1.v3_ = can_tri.v3_;
    Triangle tri2;
    tri2.v1_ = v4;
    tri2.v2_ = can_tri.v2_;
    tri2.v3_ = can_tri.v3_;
    return std::array<Triangle, 2> { tri1, tri2 };
  }

  bool intersects(const bh::Ray<FloatT>& ray,
                  FloatT* ray_t = nullptr,
                  const FloatT t_min = 0,
                  const FloatT t_max = std::numeric_limits<FloatT>::max(),
                  const bool only_front_faces = false,
                  const FloatT tolerance = std::numeric_limits<FloatT>::epsilon()) const {
    return intersects(bh::RayData<FloatT>(ray), ray_t, t_min, t_max, only_front_faces, tolerance);
  }

  bool intersects(const bh::RayData<FloatT>& ray,
                  FloatT* ray_t = nullptr,
                  const FloatT t_min = 0,
                  const FloatT t_max = std::numeric_limits<FloatT>::max(),
                  const bool only_front_faces = false,
                  const FloatT tolerance = std::numeric_limits<FloatT>::epsilon()) const {
    const RayIntersectionType intersection = intersect(ray, t_min, t_max, only_front_faces, tolerance);
    if (intersection.intersects) {
      if (ray_t != nullptr) {
        ray_t = intersection.ray_t;
      }
      return true;
    }
    else {
      return false;
    }
  }

  RayIntersectionType intersect(const bh::Ray<FloatT>& ray,
                                const FloatT t_min = 0,
                                const FloatT t_max = std::numeric_limits<FloatT>::max(),
                                const bool only_front_faces = false,
                                const FloatT tolerance = std::numeric_limits<FloatT>::epsilon()) const {
    return intersect(bh::RayData<FloatT>(ray), t_min, t_max, only_front_faces, tolerance);
  }

  RayIntersectionType intersect(const bh::RayData<FloatT>& ray,
                                const FloatT t_min = 0,
                                const FloatT t_max = std::numeric_limits<FloatT>::max(),
                                const bool only_front_faces = false,
                                const FloatT tolerance = std::numeric_limits<FloatT>::epsilon()) const {
    // Moeller-Trumbore intersection
    // (see https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm and
    //  http://www.cs.virginia.edu/~gfx/Courses/2003/ImageSynthesis/papers/Acceleration/Fast%20MinimumStorage%20RayTriangle%20Intersection.pdf)
    const Vector3 e1 = v2_ - v1_;
    const Vector3 e2 = v3_ - v1_;
    const Vector3 p = ray.direction.cross(e2);
    const FloatT det = e1.dot(p);
    if (only_front_faces) {
      if (det < tolerance) {
        return RayIntersectionType(false, 0);
      }
    }
    else {
      if (det > -tolerance && det < tolerance) {
        return RayIntersectionType(false, 0);
      }
    }
    const FloatT inv_det = 1 / det;
    const Vector3 vt = ray.origin - v1_;
    const FloatT u = vt.dot(p) * inv_det;
    if (u < 0 || u > 1) {
      return RayIntersectionType(false, 0);
    }
    const Vector3 q = vt.cross(e1);
    const FloatT v = ray.direction.dot(q) * inv_det;
    if (v < 0 || u + v > 1) {
      return RayIntersectionType(false, 0);
    }
    const FloatT t = e2.dot(q) * inv_det;
    if (t >= t_min && t <= t_max) {
      return RayIntersectionType(true, t);
    }
    else {
      return RayIntersectionType(false, 0);
    }
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Compute height of triangle given the base vector and another vector
  FloatType computeTriangleHeightSquare(const Vector3& v_base, const Vector3& v_other) const {
    Vector3 v_proj = projectVector(v_other, v_base);
    FloatType height_square = (v_other - v_proj).squaredNorm();
    return height_square;
  }

  union {
    struct {
      Vector3 v1_;
      Vector3 v2_;
      Vector3 v3_;
    };
    std::array<Vector3, 3> v_;
  };
};

template <typename FloatT>
class Polygon2D {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType);

  Polygon2D() {}

  explicit Polygon2D(const std::vector<Vector2>& vertices)
  : vertices_(vertices) {}

  bool isValid() const {
    // TODO: Make this consistent by either allowing single points and lines
    // or also reporting colinear vertices as invalid
    return getVertexCount() >= 3;
  }

  std::size_t getVertexCount() const {
    return vertices_.size();
  }

  const Vector2& getVertex(std::size_t index) const {
    return vertices_[index];
  }

  const std::vector<Vector2>& getVertices() const {
    return vertices_;
  }

  const bool isPointOutside(const Vector2& point) const {
    return !isPointInside(point);
  }

  const bool isPointInside(const Vector2& point) const {
    // Crossing test by 'shooting' a ray in the +x direction
    bool intersects = false;
    for (auto it = vertices_.begin(); it != vertices_.end(); ++it) {
      auto next_it = it + 1;
      if (next_it == vertices_.end()) {
        next_it = vertices_.begin();
      }
      Vector2 vertex1 = *it - point;
      Vector2 vertex2 = *next_it - point;
      if ((vertex1(1) < 0 && vertex2(1) >= 0)
          || (vertex1(1) >= 0 && vertex2(1) < 0)) {
        // Intersection is possible
        // TODO: Make sure this is correct even if the ray is going through a vertex (should give 2 counts)
        if ((vertex1(0) > 0 && vertex2(0) > 0)) {
          // Ray intersects this edge
          intersects = !intersects;
        }
        else if (!(vertex1(0) < 0 && vertex2(0) < 0)) {
          // Check intersection
          FloatType inv_slope = (vertex2(0) - vertex1(0)) / (vertex2(1) - vertex1(1));
          FloatType x_intersection = vertex1(0) -vertex1(1) * inv_slope;
          if (x_intersection >= 0) {
            intersects = !intersects;
          }
        }
      }
    }
    return intersects;
  }

  FloatType distanceToPoint(const Vector3& point) const {
    return std::sqrt(squareDistanceToPoint(point));
  }

  FloatType squareDistanceToPoint(const Vector2& point) const {
    FloatType dist_sq = std::numeric_limits<FloatType>::max();
    for (auto it = vertices_.begin(); it != vertices_.end(); ++it) {
      auto next_it = it + 1;
      if (next_it == vertices_.end()) {
        next_it = vertices_.begin();
      }
      const Vector2 line_segment = *next_it - *it;
      const Vector2 norm_line_segment = line_segment.normalized();
      const Vector2 point_vector = point - *it;
      FloatType dot_prod = point_vector.dot(norm_line_segment);
      dot_prod = std::max<FloatType>(dot_prod, 0);
      Vector2 orthogonal_vector;
      if (dot_prod * dot_prod > line_segment.squaredNorm()) {
        orthogonal_vector = point_vector - line_segment;
      }
      else {
        const Vector2 parallel_segment = norm_line_segment * dot_prod;
        orthogonal_vector = point_vector - parallel_segment;
      }
      dist_sq = std::min(dist_sq, orthogonal_vector.squaredNorm());
    }
    return dist_sq;
  }

  Vector2 getCentroid() const {
    Vector2 centroid(0, 0);
    for (auto it = vertices_.begin(); it != vertices_.end(); ++it) {
      centroid += *it;
    }
    return centroid / getVertexCount();
  }

  FloatType getEnclosingRadius() const {
    return std::sqrt(getEnclosingRadiusSquare());
  }

  FloatType getEnclosingRadiusSquare() const {
    FloatType radius_sq = 0;
    const Vector2 centroid = getCentroid();
    for (auto it = vertices_.begin(); it != vertices_.end(); ++it) {
      radius_sq = std::max(radius_sq, (*it - centroid).squaredNorm());
    }
    return radius_sq;
  }

  Vector2 getMinimum() const {
    Vector2 v_min(std::numeric_limits<FloatType>::max(), std::numeric_limits<FloatType>::max());
    for (auto it = vertices_.begin(); it != vertices_.end(); ++it) {
      v_min = v_min.cwiseMin(*it);
    }
    return v_min;
  }

  Vector2 getMaximum() const {
    Vector2 v_max(std::numeric_limits<FloatType>::lowest(), std::numeric_limits<FloatType>::lowest());
    for (auto it = vertices_.begin(); it != vertices_.end(); ++it) {
      v_max = v_max.cwiseMax(*it);
    }
    return v_max;
  }

private:
  std::vector<Vector2> vertices_;
};

template <typename FloatT>
class PolygonWithLowerAndUpperPlane {
public:
  using FloatType = FloatT;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using Polygon2DType = Polygon2D<FloatType>;
  using BoundingBoxType = BoundingBox3D<FloatType>;

  PolygonWithLowerAndUpperPlane()
  : lower_plane_z_(1), upper_plane_z_(-1) {}

  PolygonWithLowerAndUpperPlane(const Polygon2DType& polygon2d, FloatType lower_plane_z, FloatType upper_plane_z)
  : polygon2d_(polygon2d), lower_plane_z_(lower_plane_z), upper_plane_z_(upper_plane_z) {}

  PolygonWithLowerAndUpperPlane(const std::vector<Vector2>& vertices2d, FloatType lower_plane_z, FloatType upper_plane_z)
  : polygon2d_(vertices2d), lower_plane_z_(lower_plane_z), upper_plane_z_(upper_plane_z) {}

  bool isValid() const {
    return (upper_plane_z_ >= lower_plane_z_) && polygon2d_.isValid();
  }

  const Polygon2DType& getPolygon2D() const {
    return polygon2d_;
  }

  FloatType getLowerPlaneZ() const {
    return lower_plane_z_;
  }

  FloatType getUpperPlaneZ() const {
    return upper_plane_z_;
  }

  bool isPointOutside(const Vector3& point) const {
    return !isPointInside(point);
  }

  bool isPointInside(const Vector3& point) const {
    const bool inside_z = point(2) >= lower_plane_z_ && point(2) <= upper_plane_z_;
    if (inside_z) {
      return polygon2d_.isPointInside(point.topRows(2));
    }
    else {
      return false;
    }
  }

  bool isPointOutsideZ(const Vector3& point) const {
    return !isPointInsideZ(point);
  }

  bool isPointInsideZ(const Vector3& point) const {
    const bool inside_z = point(2) >= lower_plane_z_ && point(2) <= upper_plane_z_;
    return inside_z;
  }

  FloatType distanceToPoint(const Vector3& point) const {
    return std::sqrt(squareDistanceToPoint(point));
  }

  FloatType squareDistanceToPoint(const Vector3& point) const {
    FloatType dist_sq_xy;
    if (polygon2d_.isPointInside(point.topRows(2))) {
      dist_sq_xy = 0;
    }
    else {
      dist_sq_xy = polygon2d_.squareDistanceToPoint(point.topRows(2));
    }
    FloatType dist_z = 0;
    if (isPointOutsideZ(point)) {
      dist_z = std::min(std::abs(point(2) - lower_plane_z_), std::abs(point(2) - upper_plane_z_));
    }
    return dist_sq_xy + dist_z * dist_z;
  }

  Vector3 getCentroid() const {
    const Vector2 centroid2d = polygon2d_.getCentroid();
    const FloatType center_z = (lower_plane_z_ + upper_plane_z_) / 2;
    Vector3 centroid;
    centroid.topRows(2) = centroid2d;
    centroid(2) = center_z;
    return centroid;
  }

  FloatType getEnclosingRadius() const {
    return std::sqrt(getEnclosingRadiusSquare());
  }

  FloatType getEnclosingRadiusSquare() const {
    const FloatType radius_z = (upper_plane_z_ - lower_plane_z_) / 2;
    return polygon2d_.getEnclosingRadiusSquare() + radius_z * radius_z;
  }

  BoundingBoxType getBoundingBox() const {
    Vector2 min2d = polygon2d_.getMinimum();
    Vector2 max2d = polygon2d_.getMaximum();
    Vector3 min3d(min2d(0), min2d(1), lower_plane_z_);
    Vector3 max3d(max2d(0), max2d(1), upper_plane_z_);
    return BoundingBoxType(min3d, max3d);
  }

private:
  Polygon2DType polygon2d_;
  FloatType lower_plane_z_;
  FloatType upper_plane_z_;
};

#if __GNUC__ && !__CUDACC__
  #pragma GCC pop_options
#endif

}
