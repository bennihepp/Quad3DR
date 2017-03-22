//==================================================
// triangle_mesh.h
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 20, 2016
//==================================================

#pragma once

#include <memory>
#include <vector>
#include <unordered_map>
#include <Eigen/Dense>

#include "../memory.h"
#include "../eigen.h"
#include "../color.h"
#include "../math/geometry.h"

namespace bh {

template <typename FloatT, typename IndexT = size_t>
class TriangleMeshFactory;

template <typename FloatT, typename IndexT = size_t>
class TriangleMesh {
public:
  using FloatType = FloatT;
  using IndexType = IndexT;
  using ColorType = Color4<FloatT>;
  using TriangleType = Triangle<FloatT>;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using Vector3i = Eigen::Matrix<IndexType, 3, 1>;

  explicit TriangleMesh();

  explicit TriangleMesh(const std::vector<Vector3i>& triangle_vertex_indices,
                        const std::vector<Vector3>& vertices);

  explicit TriangleMesh(const std::vector<Vector3i>& triangle_vertex_indices,
                        const std::vector<Vector3>& vertices,
                        const std::vector<Vector3>& normals);

  explicit TriangleMesh(const std::vector<Vector3i>& triangle_vertex_indices,
                        const std::vector<Vector3>& vertices,
                        const std::vector<ColorType>& colors);

  explicit TriangleMesh(const std::vector<Vector3i>& triangle_vertex_indices,
                        const std::vector<Vector3>& vertices,
                        const std::vector<Vector3>& normals,
                        const std::vector<ColorType>& colors);

  explicit TriangleMesh(const std::vector<Vector3i>& triangle_vertex_indices,
                        const std::vector<Vector3>& vertices,
                        const std::vector<Vector3>& normals,
                        const std::vector<ColorType>& colors,
                        const std::vector<Vector2>& texture_uvs);

  virtual ~TriangleMesh();

  const std::vector<Vector3i>& triangleVertexIndices() const;

  std::vector<Vector3i>& triangleVertexIndices();

  const std::vector<Vector3>& vertices() const;

  std::vector<Vector3>& vertices();

  bool hasNormals() const;

  const std::vector<Vector3>& normals() const;

  std::vector<Vector3>& normals();

  bool hasTextureUVs() const;

  const std::vector<Vector2>& textureUVs() const;

  std::vector<Vector2>& textureUVs();

  bool hasColors() const;

  const std::vector<ColorType>& colors() const;

  std::vector<ColorType>& colors();

  TriangleType getTriangle(const IndexType idx) const;

  std::vector<TriangleType> getTriangles() const;

  enum SubdivisionStrategy {
    SUBDIVIDE_BASE_2,  // Subdivide each triangle at the midpoint of it's longest side -> results in 2 triangles
    SUBDIVIDE_MIDPOINTS_4,  // Subdivide each triangle at the midpoints of all sides -> results n 4 triangles
  };

  /// Subdivides all triangles once. Returns number of subdivisions.
  size_t subdivideTriangles(const SubdivisionStrategy sub_strategy = SUBDIVIDE_MIDPOINTS_4);

  /// Subdivides triangles until they are smaller than max_triangle_area. Returns number of subdivisions.
  size_t subdivideTrianglesUntilMaxArea(const FloatT max_triangle_area,
                                        const SubdivisionStrategy sub_strategy = SUBDIVIDE_MIDPOINTS_4);

  /// Subdivides triangles once if predicate evaluates to true (predicate must take a TriangleType). Returns number of subdivisions.
  template <typename UnaryPredicate>
  size_t subdivideTrianglesIf(const UnaryPredicate pred,
                              const SubdivisionStrategy sub_strategy = SUBDIVIDE_MIDPOINTS_4);

  /// Subdivides all triangles until predicate evaluates to true (predicate must take a TriangleType). Returns number of subdivisions.
  template <typename UnaryPredicate>
  size_t subdivideTrianglesUntil(const UnaryPredicate pred,
                                 const SubdivisionStrategy sub_strategy = SUBDIVIDE_MIDPOINTS_4);

private:
  friend class TriangleMeshFactory<FloatT, IndexT>;

  std::vector<Vector3i> triangle_vertex_indices_;
  std::vector<Vector3> vertices_;
  std::vector<Vector3> normals_;
  std::vector<Vector2> texture_uvs_;
  std::vector<ColorType> colors_;
};

template <typename FloatT, typename IndexT>
class TriangleMeshFactory {
public:
  using FloatType = FloatT;
  using IndexType = IndexT;
  using ColorType = Color4<FloatT>;
  using TriangleType = Triangle<FloatT>;
  USE_FIXED_EIGEN_TYPES(FloatType);
  using TriangleMeshType = TriangleMesh<FloatType, IndexType>;
  using Vector3i = typename TriangleMeshType::Vector3i;

  using SubdivisionStrategy = typename TriangleMeshType::SubdivisionStrategy;

  TriangleMeshFactory() = delete;
  TriangleMeshFactory(const TriangleMeshFactory&) = delete;
  TriangleMeshFactory& operator=(const TriangleMeshFactory&) = delete;

  /// Create a triangle with base along x-axis from x = -1 to x = +1 and top at y = 1 (z = 0 for all vertices)
  static TriangleMeshType createCanonicalTriangle();

  /// Create a icosahedron mesh
  static TriangleMeshType createIcosahedron(const FloatT radius, const bool generate_normals = true);

  /// Create a mesh of a prism
  static TriangleMeshType createTriangularPrism(const FloatT height, FloatT const radius);

  /// Create a sphere mesh by subdividing an icosahedron
  static TriangleMeshType createSphere(
          const FloatT radius, const size_t subdivisions = 10,
          const SubdivisionStrategy sub_strategy = SubdivisionStrategy::SUBDIVIDE_MIDPOINTS_4);

  /// Create a unit sphere mesh by subdividing an icosahedron
  static TriangleMeshType createUnitSphere(
          const size_t subdivisions = 10,
          const SubdivisionStrategy sub_strategy = SubdivisionStrategy::SUBDIVIDE_MIDPOINTS_4);

  /// Create a unit sphere mesh by subdividing an icosahedron (cached with weak_ptr so that only one copy is in memory)
  static std::shared_ptr<TriangleMeshType> createSharedUnitSphere(
          const size_t subdivisions = 10,
          const SubdivisionStrategy sub_strategy = SubdivisionStrategy::SUBDIVIDE_MIDPOINTS_4);

  /// Create a cylinder mesh by subdividing a triangle for the bottom and top cap
  static TriangleMeshType createCylinder(const FloatT height, const FloatT radius, const size_t subdivisions = 10);

  /// Create a unit radius cylinder mesh by subdividing a triangle for the bottom and top cap
  static TriangleMeshType createUnitRadiusCylinder(const FloatT cylinder_length, const size_t subdivisions = 10);

  /// Create a unit radius cylinder mesh by subdividing a triangle for the bottom and top cap
  /// (cached with weak_ptr so that only one copy is in memory)
  static std::shared_ptr<TriangleMeshType> createSharedUnitRadiusCylinder(
          const FloatT cylinder_length, const size_t subdivisions = 10);

  /// Create a mesh from a range of triangles of type Triangle<FloatT>
  template <typename TriIterator>
  static TriangleMeshType createFromTriangles(const TriIterator first, const TriIterator last);

private:
  using CacheStorageUnitSphere = CacheStorage<
          std::pair<size_t, SubdivisionStrategy>,
          TriangleMeshType,
          bh::pair_hash<size_t, SubdivisionStrategy>>;

  static CacheStorageUnitSphere unit_sphere_storage_;

//  static CacheStorage<std::pair<FloatT, size_t>, TriangleMeshType, bh::pair_hash<FloatT, size_t>>
//    unit_radius_cylinder_storage_;
    ;
};

} /* namespace bh */

#include "triangle_mesh.hxx"
