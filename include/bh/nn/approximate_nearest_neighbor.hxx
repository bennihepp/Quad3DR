//==================================================
// approximate_nearest_neighbor.hxx.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 21.03.17

namespace bh {

#pragma GCC push_options
#pragma GCC optimize("O0")

template<typename FloatT, std::size_t dimension, typename NormT>
ApproximateNearestNeighbor<FloatT, dimension, NormT>::ApproximateNearestNeighbor(
        std::size_t num_trees, NormT norm)
        : initialized_(false), index_params_(flann::KDTreeIndexParams(num_trees)),
          norm_(norm), index_(index_params_, norm_),
          points_size_(0) {
  setSearchParams(getDefaultSearchParams());
}

template<typename FloatT, std::size_t dimension, typename NormT>
ApproximateNearestNeighbor<FloatT, dimension, NormT>::ApproximateNearestNeighbor(
        const flann::IndexParams &index_params, NormT norm)
        : initialized_(false), index_params_(index_params),
          norm_(norm), index_(index_params, norm),
          points_size_(0) {
  setSearchParams(getDefaultSearchParams());
}

template<typename FloatT, std::size_t dimension, typename NormT>
ApproximateNearestNeighbor<FloatT, dimension, NormT>::~ApproximateNearestNeighbor() {
  clear();
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::getDefaultSearchParams()
-> flann::SearchParams {
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

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::clear() {
  initialized_ = false;
  index_ = flann::Index<NormT>(index_params_, norm_);
  init_points_.clear();
//  for (FlannElementType *point_ptr : points_) {
//    delete[] point_ptr;
//  }
  points_.clear();
  points_size_ = 0;
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::setSearchParams(const flann::SearchParams &params) {
  search_params_ = params;
}

// Initialize from a container of Eigen column vectors
template<typename FloatT, std::size_t dimension, typename NormT>
template<typename Iterator>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::initIndex(Iterator begin, Iterator end) {
  init_points_.reserve((end - begin) * dimension);
  for (Iterator it = begin; it != end; ++it) {
    FlannPointType flann_point;
    for (std::size_t col = 0; col < dimension; ++col) {
      flann_point[col] = (*it)(col);
    }
    init_points_.push_back(flann_point);
//    for (std::size_t col = 0; col < dimension; ++col) {
//      init_points_.push_back((*it)(col));
//    }
  }
  FlannMatrix flann_points(&(init_points_.front())[0], init_points_.size(), dimension);
  index_.buildIndex(flann_points);
  initialized_ = true;
}

template<typename FloatT, std::size_t dimension, typename NormT>
template<typename Iterator>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPoints(
        Iterator begin, Iterator end, FloatType rebuild_threshold) {
  for (Iterator it = begin; it != end; ++it) {
    addPoint(*it);
  }
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPoint(
        const Point &point, FloatType rebuild_threshold) {
  if (!initialized_) {
    initIndex(&point, &point + 1);
    return;
  }
  FlannPointType& flann_point = addPointInternal(point);
  FlannMatrix flann_mat(&flann_point[0], 1, dimension);
  index_.addPoints(flann_mat, (float)rebuild_threshold);
  ++points_size_;
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::getPoint(std::size_t point_id) const -> Point {
  // FLANN is not supposed to modify anything
  flann::Index <NormT> &index_const = const_cast<flann::Index <NormT> &>(index_);
  const FlannElementType *flann_point = index_const.getPoint(point_id);
  Point point;
  for (std::size_t i = 0; i < dimension; ++i) {
    point(i) = static_cast<FloatType>(flann_point[i]);
  }
  return point;
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearch(
        const Point &point, std::size_t knn,
        std::vector <IndexType> *indices, std::vector <DistanceType> *distances) const {
  if (empty()) {
    indices->resize(0);
    distances->resize(0);
    return;
  }
  // FLANN is not supposed to modify the values
  Point &point_nonconst = const_cast<Point &>(point);
  FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
  BH_ASSERT(indices->size() == knn);
  BH_ASSERT(distances->size() == knn);
  FlannIndexMatrix flann_indices(indices->data(), 1, knn);
  FlannMatrix flann_distances(distances->data(), 1, knn);
  int count = index_.knnSearch(flann_query, flann_indices, flann_distances, knn, search_params_);
  BH_ASSERT((std::size_t) count <= knn);
  indices->resize(count);
  distances->resize(count);
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearch(
        const Point &point, std::size_t knn) const -> SingleResult {
  if (empty()) {
    return SingleResult();
  }
  // FLANN is not supposed to modify the values
  Point &point_nonconst = const_cast<Point &>(point);
  Result result;
  FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
  int count = index_.knnSearch(flann_query, result.indices, result.distances, knn, search_params_);
  BH_ASSERT((std::size_t) count == result.indices.size());
  SingleResult single_result;
  single_result.indices = std::move(result.indices.first());
  single_result.distances = std::move(result.distances.first());
  return single_result;
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearch(
        const EigenMatrix &points, std::size_t knn) const -> Result {
  if (empty()) {
    return Result();
  }
  // FLANN is not supposed to modify the values
  EigenMatrix &points_nonconst = const_cast<EigenMatrix &>(points);
  FlannMatrix flann_queries(points_nonconst.data(), points_nonconst.rows(), points_nonconst.cols());
  Result result;
  int count = index_.knnSearch(flann_queries, result.indices, result.distances, knn, search_params_);
  BH_ASSERT((std::size_t) count == result.indices.size());
  return result;
}

template<typename FloatT, std::size_t dimension, typename NormT>
template<typename Iterator>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearch(
        Iterator begin, Iterator end, std::size_t knn) const -> Result {
  std::size_t num_points = end - begin;
  EigenMatrix points(num_points, dimension);
  for (Iterator it = begin; it != end; ++it) {
    points.row(it - begin) = it->transpose();
  }
  return knnSearch(points, knn);
}

template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearch(
        const Point &point, FloatType radius, std::size_t max_results,
        std::vector <IndexType> *indices, std::vector <DistanceType> *distances) const {
  if (empty()) {
    indices->resize(0);
    distances->resize(0);
    return;
  }
  // FLANN is not supposed to modify the values
  Point &point_nonconst = const_cast<Point &>(point);
  FlannMatrix flann_query(point_nonconst.data(), 1, point_nonconst.rows());
  indices->resize(max_results);
  distances->resize(max_results);
  FlannIndexMatrix flann_indices(indices->data(), 1, max_results);
  FlannMatrix flann_distances(distances->data(), 1, max_results);
  int count = index_.radiusSearch(flann_query, flann_indices, flann_distances, static_cast<float>(radius),
                                  search_params_);
  indices->resize(count);
  distances->resize(count);
}

/// Only for testing against knnSearch
template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::knnSearchExact(
        const Point &point, std::size_t knn,
        std::vector <IndexType> *indices, std::vector <DistanceType> *distances) const {
  using std::swap;
  knn = std::min(knn, numPointsInternal());
  indices->resize(knn);
  distances->resize(knn);
  for (std::size_t i = 0; i < knn; ++i) {
    (*indices)[i] = static_cast<IndexType>(-1);
    (*distances)[i] = std::numeric_limits<DistanceType>::max();
  }
  for (std::size_t i = 0; i < init_points_.size(); ++i) {
    const FlannPointType& flann_point = init_points_[i];
    FloatType dist_square = 0;
    for (std::size_t col = 0; col < dimension; ++col) {
      FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
      dist_square += d * d;
    }
    if (dist_square < (*distances)[distances->size() - 1]) {
      (*distances)[distances->size() - 1] = dist_square;
      (*indices)[distances->size() - 1] = i;
    }
    // Fix ordering of nearest neighbors
    for (std::size_t j = distances->size() - 1; j > 0; --j) {
      if ((*distances)[j - 1] > (*distances)[j]) {
        swap((*distances)[j - 1], (*distances)[j]);
        swap((*indices)[j - 1], (*indices)[j]);
      }
    }
  }
  size_t i = init_points_.size();
  for (auto it = std::begin(points_); it != std::end(points_); ++it, ++i) {
    const FlannPointType& flann_point = *it;
    FloatType dist_square = 0;
    for (std::size_t col = 0; col < dimension; ++col) {
      FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
      dist_square += d * d;
    }
    if (dist_square < (*distances)[distances->size() - 1]) {
      (*distances)[distances->size() - 1] = dist_square;
      (*indices)[distances->size() - 1] = i;
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

/// Only for testing against radiusSearch
template<typename FloatT, std::size_t dimension, typename NormT>
void ApproximateNearestNeighbor<FloatT, dimension, NormT>::radiusSearchExact(
        const Point &point, FloatType radius, std::size_t max_results,
        std::vector <IndexType> *indices, std::vector <DistanceType> *distances) const {
  indices->resize(0);
  distances->resize(0);
  for (std::size_t i = 0; i < init_points_.size(); ++i) {
    const FlannPointType& flann_point = init_points_[i];
    FloatType dist_square = 0;
    for (std::size_t col = 0; col < dimension; ++col) {
      FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
      dist_square += d * d;
    }
    if (dist_square <= radius) {
      indices->push_back(i);
      distances->push_back(dist_square);
      if (indices->size() >= max_results) {
        return;
      }
    }
  }
  size_t i = init_points_.size();
  for (auto it = std::begin(points_); it != std::end(points_); ++it) {
    const FlannPointType& flann_point = *it;
    FloatType dist_square = 0;
    for (std::size_t col = 0; col < dimension; ++col) {
      FloatType d = point(col) - static_cast<FloatType>(flann_point[col]);
      dist_square += d * d;
    }
    if (dist_square <= radius) {
      indices->push_back(i);
      distances->push_back(dist_square);
      if (indices->size() >= max_results) {
        return;
      }
    }
  }
  if (search_params_.sorted) {
    std::vector<size_t> idx_array;
    for (size_t i = 0; i < indices->size(); ++i) {
      idx_array.push_back(i);
    }
    std::sort(idx_array.begin(), idx_array.end(), [&](const size_t a, const size_t b) {
      return distances[a] < distances[b];
    });
    std::vector<IndexType> indices_copy = *indices;
    std::vector<DistanceType> distances_copy = *distances;
    for (size_t i = 0; i < indices->size(); ++i) {
      (*indices)[i] = indices_copy[idx_array[i]];
      (*distances)[i] = distances_copy[idx_array[i]];
    }
  }
}

template<typename FloatT, std::size_t dimension, typename NormT>
bool ApproximateNearestNeighbor<FloatT, dimension, NormT>::empty() const {
  return numPoints() == 0;
}

template<typename FloatT, std::size_t dimension, typename NormT>
std::size_t ApproximateNearestNeighbor<FloatT, dimension, NormT>::numPoints() const {
  return numPointsInternal();
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::addPointInternal(const Point &point) -> FlannPointType& {
  FlannPointType flann_point;
  for (std::size_t col = 0; col < dimension; ++col) {
    flann_point[col] = point(col);
  }
  points_.push_front(flann_point);
  return points_.front();
//  FlannElementType *point_ptr = new FlannElementType[dimension];
//  for (std::size_t col = 0; col < dimension; ++col) {
//    point_ptr[col] = point(col);
//  }
//  points_.push_front();
//  return point_ptr;
}

template<typename FloatT, std::size_t dimension, typename NormT>
std::size_t ApproximateNearestNeighbor<FloatT, dimension, NormT>::numPointsInternal() const {
  return init_points_.size() + points_size_;
}

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::getFlannIndex() const -> const flann::Index<NormT>& {
  return index_;
};

template<typename FloatT, std::size_t dimension, typename NormT>
auto ApproximateNearestNeighbor<FloatT, dimension, NormT>::getFlannIndex() -> flann::Index<NormT>& {
  return index_;
};

}

#pragma GCC pop_options