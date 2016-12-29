/*
 * approximate_nearest_neighbor.h
 *
 *  Created on: Dec 29, 2016
 *      Author: bhepp
 */
#pragma once

#include <flann/flann.hpp>
#include <ait/common.h>
#include <ait/eigen.h>

template <typename FloatT, std::size_t dimension, typename NormT = flann::L2<FloatT>>
class ApproximateNearestNeighbor {
public:
  using FloatType = FloatT;
  using Point = Eigen::Matrix<FloatType, dimension, 1>;
  using IndexType = std::size_t;
  using DistanceType = typename NormT::ResultType;
  using FlannMatrix = flann::Matrix<FloatType>;
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
  }

  void setSearchParams(const flann::SearchParams& params) {
    search_params_ = params;
  }

  // Initialize from an Eigen matrix with size num_points x dimension
  void initIndex(const EigenMatrix& points) {
    // FLANN is not supposed to modify the values
    EigenMatrix& points_nonconst = const_cast<EigenMatrix&>(points);
    FlannMatrix flann_points(points_nonconst.data(), points_nonconst.rows(), points_nonconst.cols());
    index_.buildIndex(flann_points);
    initialized_ = true;
  }

  // Initialize from a container of Eigen column vectors
  template <typename Iterator>
  void initIndex(Iterator begin, Iterator end) {
    std::size_t num_points = end - begin;
    EigenMatrix points(num_points, dimension);
    for (Iterator it = begin; it != end; ++it) {
      points.row(it - begin) = it->transpose();
    }
    initIndex(points);
  }

  void addPoints(const EigenMatrix& points, FloatType rebuild_threshold = 2) {
    if (!initialized_) {
      initIndex(points);
      return;
    }
    // FLANN is not supposed to modify the values
    EigenMatrix& points_nonconst = const_cast<EigenMatrix&>(points);
    FlannMatrix flann_points(points.data(), points.rows(), points.cols());
    index_.addPoints(flann_points, (float)rebuild_threshold);
  }

  template <typename Iterator>
  void addPoints(Iterator begin, Iterator end, FloatType rebuild_threshold = 2) {
    std::size_t num_points = end - begin;
    EigenMatrix points(num_points, dimension);
    for (Iterator it = begin; it != end; ++it) {
      points.row(it - begin) = it->transpose();
    }
    addPoints(points, rebuild_threshold);
  }

  void addPoint(const Point& point, FloatType rebuild_threshold = 2) {
    if (!initialized_) {
      initIndex(&point, &point + 1);
      return;
    }
    // FLANN is not supposed to modify the values
    Point& point_nonconst = const_cast<Point&>(point);
    FlannMatrix flann_point(point_nonconst.data(), 1, point_nonconst.rows());
    index_.addPoints(flann_point, (float)rebuild_threshold);
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

private:
  bool initialized_;
  flann::IndexParams index_params_;
  NormT norm_;
  flann::Index<NormT> index_;
  flann::SearchParams search_params_;
};
