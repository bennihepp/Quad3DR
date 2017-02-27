/*
 * approximate_nearest_neighbor.h
 *
 *  Created on: Dec 29, 2016
 *      Author: bhepp
 */
#pragma once

#include <algorithm>
#include <flann/flann.hpp>
#include <ait/common.h>
#include <ait/eigen.h>

namespace ait {

template <typename FloatT, std::size_t dimension, typename NormT = flann::L2<FloatT>>
class ApproximateNearestNeighbor {
public:
  using FloatType = FloatT;
  using Point = Eigen::Matrix<FloatType, dimension, 1>;
  using PointNotAligned = Eigen::Matrix<FloatType, dimension, 1, Eigen::DontAlign>;
  using IndexType = std::size_t;
  using DistanceType = typename NormT::ResultType;
  using FlannElementType = typename NormT::ElementType;
  using FlannMatrix = flann::Matrix<FlannElementType>;
  using FlannIndexMatrix = flann::Matrix<std::size_t>;
  using FlannDistanceMatrix = flann::Matrix<DistanceType>;
  using EigenMatrix = Eigen::Matrix<FloatType, Eigen::Dynamic, Eigen::Dynamic>;

  ApproximateNearestNeighbor(std::size_t num_trees = 4, NormT norm = NormT())
  : initialized_(false), index_params_(flann::KDTreeIndexParams(num_trees)),
    norm_(norm), index_(index_params_, norm_) {
    setSearchParams(getDefaultSearchParams());
  }

  ApproximateNearestNeighbor(const flann::IndexParams& index_params, NormT norm = NormT())
  : initialized_(false), index_params_(index_params),
    norm_(norm),  index_(index_params, norm) {
    setSearchParams(getDefaultSearchParams());
  }

  ~ApproximateNearestNeighbor() {
    clear();
  }

  static flann::SearchParams getDefaultSearchParams() {
    flann::SearchParams params;
    params.checks = 128;
    params.eps = 0;
    params.sorted = true;
    params.max_neighbors = -1;
    params.use_heap = flann::FLANN_Undefined;
    params.cores = 1;
    params.matrices_in_gpu_ram = false;
    return params;
  }

  void clear() {
    initialized_ = false;
    index_ = flann::Index<NormT>(index_params_, norm_);
    init_points_.clear();
    for (FlannElementType* point_ptr : points_) {
      delete[] point_ptr;
    }
    points_.clear();
  }

  void setSearchParams(const flann::SearchParams& params) {
    search_params_ = params;
  }

  // Initialize from a container of Eigen column vectors
  template <typename Iterator>
  void initIndex(Iterator begin, Iterator end) {
    init_points_.reserve((end - begin) * dimension);
    for (Iterator it = begin; it != end; ++it) {
      for (std::size_t col = 0; col < dimension; ++col) {
        init_points_.push_back((*it)(col));
      }
    }
    FlannMatrix flann_points(init_points_.data(), init_points_.size() / dimension, dimension);
    index_.buildIndex(flann_points);
    initialized_ = true;
  }

  template <typename Iterator>
  void addPoints(Iterator begin, Iterator end, FloatType rebuild_threshold = 2) {
    for (Iterator it = begin; it != end; ++it) {
      addPoint(*it);
    }
  }

  void addPoint(const Point& point, FloatType rebuild_threshold = 2) {
    if (!initialized_) {
      initIndex(&point, &point + 1);
      return;
    }
    FlannElementType* point_ptr = addPointInternal(point);
    FlannMatrix flann_point(point_ptr, 1, dimension);
    index_.addPoints(flann_point, (float)rebuild_threshold);
  }

  Point getPoint(std::size_t point_id) const {
    // FLANN is not supposed to modify anything
    flann::Index<NormT>& index_const = const_cast<flann::Index<NormT>&>(index_);
    const FlannElementType* flann_point = index_const.getPoint(point_id);
    Point point;
    for (std::size_t i = 0; i < dimension; ++i) {
      point(i) = static_cast<FloatType>(flann_point[i]);
    }
    return point;
  }

  struct SingleResult {
    std::vector<IndexType> indices;
    std::vector<DistanceType> distances;
  };

  struct Result {
    std::vector<std::vector<IndexType>> indices;
    std::vector<std::vector<DistanceType>> distances;
  };

  void knnSearch(const Point& point, std::size_t knn, std::vector<IndexType>* indices, std::vector<DistanceType>* distances) {
    if (empty()) {
      indices->resize(0);
      distances->resize(0);
      return;
    }
    // FLANN is not supposed to modify the values
    Point& point_nonconst = const_cast<Point&>(point);
    FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
    AIT_ASSERT(indices->size() == knn);
    AIT_ASSERT(distances->size() == knn);
    FlannIndexMatrix flann_indices(indices->data(), 1, knn);
    FlannMatrix flann_distances(distances->data(), 1, knn);
    int count = index_.knnSearch(flann_query, flann_indices, flann_distances, knn, search_params_);
    AIT_ASSERT((std::size_t)count <= knn);
    indices->resize(count);
    distances->resize(count);
  }

  SingleResult knnSearch(const Point& point, std::size_t knn) {
    if (empty()) {
      return SingleResult();
    }
    // FLANN is not supposed to modify the values
    Point& point_nonconst = const_cast<Point&>(point);
    Result result;
    FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
    int count = index_.knnSearch(flann_query, result.indices, result.distances, knn, search_params_);
    AIT_ASSERT((std::size_t)count == result.indices.size());
    SingleResult single_result;
    single_result.indices = std::move(result.indices.first());
    single_result.distances = std::move(result.distances.first());
    return single_result;
  }

  Result knnSearch(const EigenMatrix& points, std::size_t knn) {
    if (empty()) {
      return Result();
    }
    // FLANN is not supposed to modify the values
    EigenMatrix& points_nonconst = const_cast<EigenMatrix&>(points);
    FlannMatrix flann_queries(points_nonconst.data(), points_nonconst.rows(), points_nonconst.cols());
    Result result;
    int count = index_.knnSearch(flann_queries, result.indices, result.distances, knn, search_params_);
    AIT_ASSERT((std::size_t)count == result.indices.size());
    return result;
  }

  template <typename Iterator>
  Result knnSearch(Iterator begin, Iterator end, std::size_t knn) {
    std::size_t num_points = end - begin;
    EigenMatrix points(num_points, dimension);
    for (Iterator it = begin; it != end; ++it) {
      points.row(it - begin) = it->transpose();
    }
    return knnSearch(points, knn);
  }

  void radiusSearch(const Point& point, FloatType radius, std::size_t max_results,
      std::vector<IndexType>* indices, std::vector<DistanceType>* distances) {
    if (empty()) {
      indices->resize(0);
      distances->resize(0);
      return;
    }
    // FLANN is not supposed to modify the values
    Point& point_nonconst = const_cast<Point&>(point);
    FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
    indices->resize(max_results);
    distances->resize(max_results);
    FlannIndexMatrix flann_indices(indices->data(), 1, max_results);
    FlannMatrix flann_distances(distances->data(), 1, max_results);
    int count = index_.radiusSearch(flann_query, flann_indices, flann_distances, static_cast<float>(radius), search_params_);
    indices->resize(count);
    distances->resize(count);
  }

  /// Only for testing against knnSearch
  void knnSearchExact(const Point& point, std::size_t knn, std::vector<IndexType>* indices, std::vector<DistanceType>* distances) {
    using std::swap;
    knn = std::min(knn, numPointsInternal());
    indices->resize(knn);
    distances->resize(knn);
    for (std::size_t i = 0; i < knn; ++i) {
      (*indices)[i] = static_cast<IndexType>(-1);
      (*distances)[i] = std::numeric_limits<DistanceType>::max();
    }
    for (std::size_t i = 0; i < init_points_.size(); i += dimension) {
      const FlannElementType* flann_point = &init_points_[i];
      FloatType dist_square = 0;
      for (std::size_t col = 0; col < dimension; ++col) {
        FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
        dist_square += d * d;
      }
      if (dist_square < (*distances)[distances->size() - 1]) {
        (*distances)[distances->size() - 1] = dist_square;
        (*indices)[distances->size() - 1] = i / 3;
      }
      // Fix ordering of nearest neighbors
      for (std::size_t j = distances->size() - 1; j > 0; --j) {
        if ((*distances)[j - 1] > (*distances)[j]) {
          swap((*distances)[j - 1], (*distances)[j]);
          swap((*indices)[j - 1], (*indices)[j]);
        }
      }
    }
    for (std::size_t i = 0; i < points_.size(); ++i) {
      const FlannElementType* flann_point = points_[i];
      FloatType dist_square = 0;
      for (std::size_t col = 0; col < dimension; ++col) {
        FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
        dist_square += d * d;
      }
      if (dist_square < (*distances)[distances->size() - 1]) {
        (*distances)[distances->size() - 1] = dist_square;
        (*indices)[distances->size() - 1] = init_points_.size() / dimension + i;
      }
      // Fix ordering of nearest neighbors
      for (std::size_t j = distances->size() - 1; j > 0; --j) {
        if ((*distances)[j - 1] > (*distances)[j]) {
          swap((*distances)[j - 1], (*distances)[j]);
          swap((*indices)[j - 1], (*indices)[j]);
        }
      }
    }
  }

  bool empty() const {
    return numPoints() == 0;
  }

  std::size_t numPoints() const {
    return numPointsInternal();
  }

private:
  FlannElementType* addPointInternal(const Point& point) {
    FlannElementType* point_ptr = new FlannElementType[dimension];
    for (std::size_t col = 0; col < dimension; ++col) {
      point_ptr[col] = point(col);
    }
    points_.push_back(point_ptr);
    return point_ptr;
  }

  std::size_t numPointsInternal() const {
    return init_points_.size() / dimension + points_.size();
  }

  bool initialized_;
  flann::IndexParams index_params_;
  NormT norm_;
  flann::Index<NormT> index_;
  flann::SearchParams search_params_;
  // TODO: Reorganize with better data structures (i.e. using Eigen vectors)
  // Need two arrays here because points for initialization need to be dense
  std::vector<FlannElementType> init_points_;
  std::vector<FlannElementType*> points_;
};

}
