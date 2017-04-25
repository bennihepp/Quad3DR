//==================================================
// bvh.cuh
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Jan 16, 2017
//==================================================
#pragma once

#include <utility>
#include <bh/cuda_utils.h>
#include <bh/cuda_math.h>

#if __GNUC__ && !__CUDACC__
  #pragma GCC push_options
  #pragma GCC optimize ("fast-math")
#endif

#define WITH_CUDA_RECURSION 1

namespace bvh {

template <typename FloatT>
using CudaVector2 = bh::CudaVector<FloatT, 3>;

template <typename FloatT>
using CudaVector3 = bh::CudaVector<FloatT, 3>;

template <typename FloatT>
using CudaVector4 = bh::CudaVector<FloatT, 4>;

template <typename FloatT>
using CudaMatrix3x3 = bh::CudaMatrix<FloatT, 3, 3>;

template <typename FloatT>
using CudaMatrix4x4 = bh::CudaMatrix<FloatT, 4, 4>;

template <typename FloatT>
using CudaMatrix3x4 = bh::CudaMatrix<FloatT, 3, 4>;

template <typename FloatT>
struct CudaRay {
  CudaVector3<FloatT> origin;
  CudaVector3<FloatT> direction;
};

template <typename FloatT>
struct CudaRayData : public CudaRay<FloatT> {
  CudaVector3<FloatT> inv_direction;
  FloatT min_range_sq;
  FloatT max_range_sq;
};

template <typename FloatT>
class CudaBoundingBox3D {
public:
  static CudaBoundingBox3D createFromCenterAndExtent(const CudaVector3<FloatT>& center, const CudaVector3<FloatT>& extent) {
    const CudaVector3<FloatT> min = center - extent / 2;
    const CudaVector3<FloatT> max = center + extent / 2;
    return CudaBoundingBox3D(min, max);
  }

  CudaBoundingBox3D() {
    min_(0) = 1;
    min_(1) = 1;
    min_(2) = 1;
    max_(0) = -1;
    max_(1) = -1;
    max_(2) = -1;
  }

  CudaBoundingBox3D(const CudaVector3<FloatT>& center, FloatT size)
  : min_(center.array() - size / 2), max_(center.array() + size / 2) {}

  CudaBoundingBox3D(const CudaVector3<FloatT>& min, const CudaVector3<FloatT>& max)
  : min_(min), max_(max) {}

  ~CudaBoundingBox3D() {}

  __host__ __device__
  bool operator==(const CudaBoundingBox3D& other) const {
    return min_ == other.min_ && max_ == other.max_;
  }

  __host__ __device__
  bool isValid() const {
    return (min_.array() <= max_.array()).all();
  }

  __host__ __device__
  bool isEmpty() const {
    return (min_.array() >= max_.array()).all();
  }

  __host__ __device__
  const CudaVector3<FloatT>& getMinimum() const {
    return min_;
  }

  __host__ __device__
  const FloatT getMinimum(std::size_t index) const {
    return min_(index);
  }

  __host__ __device__
  const CudaVector3<FloatT>& getMaximum() const {
    return max_;
  }

  __host__ __device__
  const FloatT getMaximum(std::size_t index) const {
    return max_(index);
  }

  __host__ __device__
  const CudaVector3<FloatT> getExtent() const {
    return max_ - min_;
  }

  __host__ __device__
  FloatT getExtent(std::size_t index) const {
    return (max_ - min_)(index);
  }

  __host__ __device__
  const FloatT getMaxExtent() const {
    return (max_ - min_).maxCoeff();
  }

  __host__ __device__
  const FloatT getMaxExtent(std::size_t* index) const {
    return (max_ - min_).maxCoeff(index);
  }

  __host__ __device__
  const CudaVector3<FloatT> getCenter() const {
    return (min_ + max_) / 2;
  }

  __host__ __device__
  const FloatT getCenter(std::size_t index) const {
    return (min_(index) + max_(index)) / 2;
  }

  __host__ __device__
  FloatT getVolume() const {
    return getExtent().array().prod();
  }

  __host__ __device__
  bool isOutside(const CudaVector3<FloatT>& point) const {
    for (std::size_t i = 0; i < point.Rows; ++i) {
      const FloatT min_value = min_(i);
      const FloatT max_value = max_(i);
      const FloatT point_value = point(i);
      const bool p_sm_min = point_value < min_value;
      const bool p_gt_max = point_value > max_value;
      if (p_sm_min || p_gt_max) {
//      if (point(i) < min_(i) || point(i) > max_(i)) {
        return true;
      }
    }
    return false;
  }

  __host__ __device__
  bool isInside(const CudaVector3<FloatT>& point) const {
    return !isOutside(point);
  }

  __host__ __device__
  bool isOutsideOf(const CudaBoundingBox3D& bbox) const {
    for (std::size_t i = 0; i < min_.Rows; ++i) {
      if (max_(i) < min_(i) || min_(i) > max_(i)) {
        return true;
      }
    }
    return false;
  }

  __host__ __device__
  bool isInsideOf(const CudaBoundingBox3D& bbox) const {
    return !isOutsideOf(bbox);
  }

  /// Return squared distance to closest point on the outside (if point is inside distance is 0)
  __host__ __device__
  FloatT squaredDistanceTo(const CudaVector3<FloatT>& point) const {
    FloatT dist_sq = 0;
    for (std::size_t i = 0; i < 3; ++i) {
      bool outside = point(i) < getMinimum(i) || point(i) > getMaximum(i);
      if (outside) {
        FloatT d = std::min(std::abs(point(i) - getMinimum(i)), std::abs(point(i) - getMaximum(i)));
        dist_sq += d * d;
      }
    }
    return dist_sq;
  }

  /// Return distance to closest point on the outside (if point is inside distance is 0)
  __host__ __device__
  FloatT distanceTo(const CudaVector3<FloatT>& point) const {
    return std::sqrt(squaredDistanceTo(point));
  }

  __host__ __device__
  bool contains(const CudaBoundingBox3D& other) const {
    return other.isInsideOf(*this);
  }

  __host__ __device__
  bool intersects(const CudaBoundingBox3D& other) const {
    if ((max_.array() < other.min_.array()).any()) {
      return false;
    }
    if ((other.max_.array() < min_.array()).any()) {
      return false;
    }
    return true;
  }

  __host__ __device__
  bool intersects(const CudaRay<FloatT>& ray, CudaVector3<FloatT>* intersection = nullptr) const {
    CudaRayData<FloatT> ray_data(ray);
    ray_data.inv_direction = ray.direction.cwiseInverse();
    return intersects(ray_data, intersection);
  }

  /// Intersect ray with bounding box.
  ///
  /// @param t_ray: Return the ray coefficient of the intersection point.
  /// @param t_min: Minimum ray equation coefficient, >= 0.
  /// @param t_max: Maximum ray equation coefficient, > t_min.
  /// @return: Returns a flag indicating whether the ray intersects the bounding box.
  __device__
  bool intersectsCuda(const CudaRayData<FloatT>& ray,
                      FloatT* t_ray,
                      const FloatT t_min = 0,
                      const FloatT t_max = std::numeric_limits<FloatT>::max()) const {
    // Faster version without explicit check for directions parallel to an axis
    FloatT t_lower = t_min;
    FloatT t_upper = t_max;
    const size_t kDimensions = 3;
    for (size_t i = 0; i < kDimensions; ++i) {
      const FloatT t0 = (min_(i) - ray.origin(i)) * ray.inv_direction(i);
      const FloatT t1 = (max_(i) - ray.origin(i)) * ray.inv_direction(i);
      t_lower = fmaxf(t_lower, fminf(t0, t1));
      t_upper = fminf(t_upper, fmaxf(t0, t1));
    }
    if (t_upper >= t_lower) {
      *t_ray = t_lower;
      return true;
    }
    else {
      *t_ray = 0;
      return false;
    }
  }

  __device__
  bool intersectsCuda(const CudaRayData<FloatT>& ray_data, CudaVector3<FloatT>* intersection = nullptr) const {
  //  std::cout << "  intersecting node at depth " << cur_depth << " with size " << node_size_half * 2 << std::endl;
    float t_min = -FLT_MAX;
    float t_max = FLT_MAX;

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
      t_min = fmaxf(t_min, fminf(t0, t1));
      t_max = fminf(t_max, fmaxf(t0, t1));
    }

    bool intersect = t_max > fmaxf(t_min, FloatT(0));
    if (intersect && intersection != nullptr) {
      t_min = fmaxf(t_min, FloatT(0));
      *intersection = ray_data.origin + ray_data.direction * t_min;
    }
    return intersect;
  }

  bool intersects(const CudaRayData<FloatT>& ray_data, CudaVector3<FloatT>* intersection = nullptr) const {
  //  std::cout << "  intersecting node at depth " << cur_depth << " with size " << node_size_half * 2 << std::endl;
    float t_min = std::numeric_limits<FloatT>::lowest();
    float t_max = std::numeric_limits<FloatT>::max();

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

    bool intersect = t_max > std::max(t_min, FloatT(0));
    if (intersect && intersection != nullptr) {
      t_min = std::max(t_min, FloatT(0));
      *intersection = ray_data.origin + ray_data.direction * t_min;
    }
    return intersect;
  }

  __host__ __device__
  CudaBoundingBox3D operator*(FloatT scale) const {
    return CudaBoundingBox3D::createFromCenterAndExtent(getCenter(), scale * getExtent());
  }

  __host__ __device__
  static CudaBoundingBox3D getUnion(const CudaBoundingBox3D& bbox_a, const CudaBoundingBox3D& bbox_b) {
    CudaVector3<FloatT> min = bbox_a.min_.cwiseMin(bbox_b.min_);
    CudaVector3<FloatT> max = bbox_a.max_.cwiseMax(bbox_b.max_);
    return CudaBoundingBox3D(min, max);
  }

  __host__ __device__
  void include(const CudaBoundingBox3D& other) {
    min_ = min_.min(other.min_);
    max_ = max_.max(other.max_);
  }

  __host__ __device__
  void include(const CudaVector3<FloatT>& point) {
    min_ = min_.min(point);
    max_ = max_.min(point);
  }

  __host__ __device__
  void constrainTo(const CudaBoundingBox3D& bbox) {
    min_ = min_.max(bbox.min_);
    max_ = max_.min(bbox.max_);
  }

private:
  CudaVector3<FloatT> min_;
  CudaVector3<FloatT> max_;
};


template <typename FloatT>
class CudaTree;

template <typename FloatT>
class CudaNode {
public:
  using BoundingBoxType = CudaBoundingBox3D<FloatT>;

  __host__ __device__
  const BoundingBoxType& getBoundingBox() const {
    return bounding_box_;
  }

  __host__ __device__
  bool hasLeftChild() const {
    return left_child_ != nullptr;
  }

  __host__ __device__
  const CudaNode* getLeftChild() const {
    return left_child_;
  }

  __host__ __device__
  CudaNode* getLeftChild() {
    return left_child_;
  }

  __host__ __device__
  bool hasRightChild() const {
    return right_child_ != nullptr;
  }

  __host__ __device__
  const CudaNode* getRightChild() const {
    return right_child_;
  }

  __host__ __device__
  CudaNode* getRightChild() {
    return right_child_;
  }

  __host__ __device__
  const void* getPtr() const {
    return ptr_;
  }

  __host__ __device__
  void* getPtr() {
    return ptr_;
  }

  __host__ __device__
  bool isLeaf() const {
    return left_child_ == nullptr && right_child_ == nullptr;
  }

  __host__ __device__
  void computeBoundingBox() {
    if (left_child_ != nullptr && right_child_ != nullptr) {
      bounding_box_ = BoundingBoxType::getUnion(left_child_->bounding_box_, right_child_->bounding_box_);
    }
    else if (left_child_ != nullptr) {
      bounding_box_ = left_child_->bounding_box_;
    }
    else if (right_child_ != nullptr) {
      bounding_box_ = right_child_->bounding_box_;
    }
  }

private:
  friend class CudaTree<FloatT>;

  BoundingBoxType bounding_box_;

  CudaNode* left_child_;
  CudaNode* right_child_;

  void* ptr_;
};

template <typename FloatT>
class CudaTree {
public:
  using NodeType = CudaNode<FloatT>;
  using CudaRayType = CudaRay<FloatT>;

  const std::size_t kThreadsPerBlock = 1024;

  struct CudaIntersectionResult {
    CudaIntersectionResult()
    : intersection(CudaVector3<FloatT>::Zero()), node(nullptr), depth(0), dist_sq(0) {}

    CudaVector3<FloatT> intersection;
    void* node;
    std::size_t depth;
    FloatT dist_sq;
  };

  struct CudaIntersectionResultWithScreenCoordinates {
    CudaIntersectionResultWithScreenCoordinates()
    : intersection_result(), screen_coordinates(CudaVector2<FloatT>::Zero()) {}

    CudaIntersectionResult intersection_result;
    CudaVector2<FloatT> screen_coordinates;
  };

  struct CudaIntersectionData {
    CudaRayData<FloatT> ray;
    FloatT min_range_sq;
  };

  struct CudaIntersectionIterativeStackEntry {
//    enum State {
//      NotVisited,
//      PushedLeftChild,
//      PushedRightChild,
//    };
    NodeType* node;
    std::size_t depth;
//    State state;
//    bool intersects;
  };

  static CudaTree* createCopyFromHostTree(NodeType* root, const std::size_t num_of_nodes, const std::size_t tree_depth);

  ~CudaTree() {
    clear();
  }

  void clear() {
    if (d_nodes_ != nullptr) {
      bh::CudaUtils::deallocate(&d_nodes_);
      d_nodes_ = nullptr;
    }
    if (d_rays_ != nullptr) {
      bh::CudaUtils::deallocate(&d_rays_);
      d_rays_ = nullptr;
    }
    if (d_results_ != nullptr) {
      bh::CudaUtils::deallocate(&d_results_);
      bh::CudaUtils::deallocate(&d_results_with_screen_coordinates_);
      d_results_ = nullptr;
    }
  }

  NodeType* getRoot() {
    return &d_nodes_[0];
  }

#if WITH_CUDA_RECURSION
  std::vector<CudaIntersectionResult> intersectsRecursive(const std::vector<CudaRayType>& rays,
      const FloatT min_range = 0, const FloatT max_range = -1);
#endif

  std::vector<CudaIntersectionResult> intersectsIterative(const std::vector<CudaRayType>& rays,
      const FloatT min_range = 0, const FloatT max_range = -1);

#if WITH_CUDA_RECURSION
  std::vector<CudaIntersectionResult> raycastRecursive(
      const CudaMatrix4x4<FloatT>& intrinsics,
      const CudaMatrix3x4<FloatT>& extrinsics,
      const std::size_t x_start, const std::size_t x_end,
      const std::size_t y_start, const std::size_t y_end,
      const FloatT min_range = 0, const FloatT max_range = -1,
      const bool fail_on_error = false);

  std::vector<CudaIntersectionResultWithScreenCoordinates> raycastWithScreenCoordinatesRecursive(
      const CudaMatrix4x4<FloatT>& intrinsics,
      const CudaMatrix3x4<FloatT>& extrinsics,
      const std::size_t x_start, const std::size_t x_end,
      const std::size_t y_start, const std::size_t y_end,
      const FloatT min_range = 0, const FloatT max_range = -1,
      const bool fail_on_error = false);
#endif

  std::vector<CudaIntersectionResult> raycastIterative(
      const CudaMatrix4x4<FloatT>& intrinsics,
      const CudaMatrix3x4<FloatT>& extrinsics,
      const std::size_t x_start, const std::size_t x_end,
      const std::size_t y_start, const std::size_t y_end,
      const FloatT min_range = 0, const FloatT max_range = -1,
      const bool fail_on_error = false);

  std::vector<CudaIntersectionResultWithScreenCoordinates> raycastWithScreenCoordinatesIterative(
          const CudaMatrix4x4<FloatT>& intrinsics,
          const CudaMatrix3x4<FloatT>& extrinsics,
          const std::size_t x_start, const std::size_t x_end,
          const std::size_t y_start, const std::size_t y_end,
          const FloatT min_range = 0, const FloatT max_range = -1,
          const bool fail_on_error = false);

private:
  CudaTree(const std::size_t tree_depth)
  : tree_depth_(tree_depth),
    d_nodes_(nullptr),
    d_rays_(nullptr), d_rays_size_(0),
    d_results_(nullptr), d_results_with_screen_coordinates_(nullptr), d_results_size_(0),
    d_stacks_(nullptr), d_stacks_size_(0) {};

  bool intersectsRecursive(const CudaIntersectionData& data,
      NodeType* cur_node, std::size_t cur_depth, CudaIntersectionResult* result);

  void reserveDeviceRaysAndResults(const std::size_t num_of_rays);

  std::size_t tree_depth_;
  NodeType* d_nodes_;
  CudaRayType* d_rays_;
  std::size_t d_rays_size_;
  CudaIntersectionResult* d_results_;
  CudaIntersectionResultWithScreenCoordinates* d_results_with_screen_coordinates_;
  std::size_t d_results_size_;
  CudaIntersectionIterativeStackEntry* d_stacks_;
  std::size_t d_stacks_size_;
};


#if __GNUC__ && !__CUDACC__
  #pragma GCC pop_options
#endif

}  // namespace bvh
