//==================================================
// triangle_mesh.hxx
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Mar 15, 2017
//==================================================

#define _USE_MATH_DEFINES
#include <cmath>
#include <array>
#include <iostream>
#include <memory>
#include <unordered_map>
#include <set>

namespace bh {

template <typename FloatT, typename IndexT>
TriangleMesh<FloatT, IndexT>::TriangleMesh() {}

template <typename FloatT, typename IndexT>
TriangleMesh<FloatT, IndexT>::TriangleMesh(
    const std::vector<Vector3i>& triangle_vertex_indices,
    const std::vector<Vector3>& vertices)
    : triangle_vertex_indices_(triangle_vertex_indices),
      vertices_(vertices) {}

template <typename FloatT, typename IndexT>
TriangleMesh<FloatT, IndexT>::TriangleMesh(
    const std::vector<Vector3i>& triangle_vertex_indices,
    const std::vector<Vector3>& vertices,
    const std::vector<Vector3>& normals)
    : triangle_vertex_indices_(triangle_vertex_indices),
      vertices_(vertices),
      normals_(normals) {}

template <typename FloatT, typename IndexT>
TriangleMesh<FloatT, IndexT>::TriangleMesh(
    const std::vector<Vector3i>& triangle_vertex_indices,
    const std::vector<Vector3>& vertices,
    const std::vector<ColorType>& colors)
    : triangle_vertex_indices_(triangle_vertex_indices),
      vertices_(vertices),
      colors_(colors) {}

template <typename FloatT, typename IndexT>
TriangleMesh<FloatT, IndexT>::TriangleMesh(
    const std::vector<Vector3i>& triangle_vertex_indices,
    const std::vector<Vector3>& vertices,
    const std::vector<Vector3>& normals,
    const std::vector<ColorType>& colors)
    : triangle_vertex_indices_(triangle_vertex_indices),
      vertices_(vertices),
      normals_(normals),
      colors_(colors) {}

template <typename FloatT, typename IndexT>
TriangleMesh<FloatT, IndexT>::TriangleMesh(
    const std::vector<Vector3i>& triangle_vertex_indices,
    const std::vector<Vector3>& vertices,
    const std::vector<Vector3>& normals,
    const std::vector<ColorType>& colors,
    const std::vector<Vector2>& texture_uvs)
    : triangle_vertex_indices_(triangle_vertex_indices),
      vertices_(vertices),
      normals_(normals),
      colors_(colors),
      texture_uvs_(texture_uvs) {}

template <typename FloatT, typename IndexT>
TriangleMesh<FloatT, IndexT>::~TriangleMesh() {}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::triangleVertexIndices() const -> const std::vector<Vector3i>& {
  return triangle_vertex_indices_;
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::triangleVertexIndices() -> std::vector<Vector3i>& {
  return triangle_vertex_indices_;
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::vertices() const -> const std::vector<Vector3>& {
  return vertices_;
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::vertices() -> std::vector<Vector3>& {
  return vertices_;
}

template <typename FloatT, typename IndexT>
bool TriangleMesh<FloatT, IndexT>::hasNormals() const {
  return !normals_.empty();
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::normals() const -> const std::vector<Vector3>& {
  return normals_;
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::normals() -> std::vector<Vector3>& {
  return normals_;
}

template <typename FloatT, typename IndexT>
bool TriangleMesh<FloatT, IndexT>::hasTextureUVs() const {
  return !texture_uvs_.empty();
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::textureUVs() const -> const std::vector<Vector2>& {
  return texture_uvs_;
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::textureUVs() -> std::vector<Vector2>& {
  return texture_uvs_;
}

template <typename FloatT, typename IndexT>
bool TriangleMesh<FloatT, IndexT>::hasColors() const {
  return !colors_.empty();
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::colors() const -> const std::vector<ColorType>& {
  return colors_;
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::colors() -> std::vector<ColorType>& {
  return colors_;
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::getTriangle(const IndexType idx) const -> TriangleType {
  const Vector3i tri_indices = triangle_vertex_indices_[idx];
  TriangleType triangle(vertices_[tri_indices[0]],
                        vertices_[tri_indices[1]],
                        vertices_[tri_indices[2]]);
  return triangle;
}

template <typename FloatT, typename IndexT>
auto TriangleMesh<FloatT, IndexT>::getTriangles() const -> std::vector<TriangleType> {
  std::vector<TriangleType> triangles;
  triangles.reserve(triangle_vertex_indices_.size());
  for (const Vector3i& tri_indices : triangle_vertex_indices_) {
    TriangleType triangle(vertices_[tri_indices[0]],
                          vertices_[tri_indices[1]],
                          vertices_[tri_indices[2]]);
    triangles.push_back(std::move(triangle));
  }
  return triangles;
}

template <typename FloatT, typename IndexT>
size_t TriangleMesh<FloatT, IndexT>::subdivideTriangles(const SubdivisionStrategy sub_strategy) {
  return subdivideTrianglesIf(
          [] (const TriangleType& triangle) {
            return true;
          },
          sub_strategy);
}

template <typename FloatT, typename IndexT>
size_t TriangleMesh<FloatT, IndexT>::subdivideTrianglesUntilMaxArea(const FloatT max_triangle_area,
                                                                    const SubdivisionStrategy sub_strategy) {
  const FloatType max_triangle_area_square = max_triangle_area * max_triangle_area;
  return subdivideTrianglesUntil(
          [&] (const TriangleType& triangle) {
            return triangle.computeTriangleAreaSquare() <= max_triangle_area_square;
          },
          sub_strategy);
}

template <typename FloatT, typename IndexT>
template <typename UnaryPredicate>
size_t TriangleMesh<FloatT, IndexT>::subdivideTrianglesIf(const UnaryPredicate pred,
                                                          const SubdivisionStrategy sub_strategy) {
  // Divide each triangle and keep track of the new vertices between old vertices (connected by edges).
  using Edge = std::pair<IndexT, IndexT>;
  using EdgeToVertexIndexMap = std::unordered_map<Edge, int, bh::hash<Edge>>;
  using EdgeToVertexIterator = typename EdgeToVertexIndexMap::iterator;
  using FindReturnType = std::pair<EdgeToVertexIterator, bool>;
  std::vector<Vector3i> new_triangle_vertex_indices;
  EdgeToVertexIndexMap edgeToVertexIndexMap;
  size_t subdivision_counter = 0;
  for (size_t i = 0; i < triangle_vertex_indices_.size(); ++i) {
    const Vector3i& tri = triangle_vertex_indices_[i];
    const IndexT vert1_index = tri(0);
    const IndexT vert2_index = tri(1);
    const IndexT vert3_index = tri(2);
    const Vector3& vert1 = vertices_[vert1_index];
    const Vector3& vert2 = vertices_[vert2_index];
    const Vector3& vert3 = vertices_[vert3_index];
    const TriangleType triangle(vert1, vert2, vert3);
    if (pred(triangle)) {
      // This lambda searches for an edge entry in the map. If there is no entry it inserts the corresponding vertex and
      // returns the new index. Edges are uniquely identified by ordering the indices.
      const auto findEdgeVertexIndexOrInsertLambda = [&] (IndexT index_a, IndexT index_b) -> IndexT {
        FindReturnType find_ret;
        if (index_a > index_b) {
          using std::swap;
          swap(index_a, index_b);
        }
        EdgeToVertexIterator vertex_iter;
        bool inserted;
        std::tie(vertex_iter, inserted) = edgeToVertexIndexMap.insert(std::make_pair(Edge(index_a, index_b), vertices_.size()));
        const IndexT found_vertex_index = vertex_iter->second;
        int vert_ab_index;
        if (inserted) {
          const Vector3& vert_a = vertices_[index_a];
          const Vector3& vert_b = vertices_[index_b];
          vert_ab_index = vertices_.size();
          vertices_.push_back((vert_a + vert_b) / 2);
          if (hasNormals()) {
            const Vector3& normal_a = normals_[index_a];
            const Vector3& normal_b = normals_[index_b];
            normals_.push_back((normal_a + normal_b).normalized());
          }
          if (hasTextureUVs()) {
            const Vector2& texture_uv_a = texture_uvs_[index_a];
            const Vector2& texture_uv_b = texture_uvs_[index_b];
            texture_uvs_.push_back((texture_uv_a + texture_uv_b) / 2);
          }
          if (hasColors()) {
            const ColorType& color_a = colors_[index_a];
            const ColorType& color_b = colors_[index_b];
            colors_.push_back((color_a + color_b) / 2);
          }
        }
        else {
          vert_ab_index = found_vertex_index;
        }
        return vert_ab_index;
      };

      switch (sub_strategy) {
      case SubdivisionStrategy::SUBDIVIDE_BASE_2:
      {
        const FloatType l1 = (vertices_[tri(1)] - vertices_[tri(0)]).squaredNorm();
        const FloatType l2 = (vertices_[tri(2)] - vertices_[tri(1)]).squaredNorm();
        const FloatType l3 = (vertices_[tri(0)] - vertices_[tri(2)]).squaredNorm();
        bool l1_is_max = false;
        bool l2_is_max = false;
        if (l1 >= l2) {
          if (l1 >= l3) {
            l1_is_max = true;
          }
        }
        else if (l2 >= l3) {
          l2_is_max = true;
        }
        if (l1_is_max) {
          int vert12_index = findEdgeVertexIndexOrInsertLambda(tri(0), tri(1));
          new_triangle_vertex_indices.push_back(Vector3i(vert1_index, vert12_index, vert3_index));
          new_triangle_vertex_indices.push_back(Vector3i(vert12_index, vert2_index, vert3_index));
        }
        else if (l2_is_max) {
          int vert23_index = findEdgeVertexIndexOrInsertLambda(tri(1), tri(2));
          new_triangle_vertex_indices.push_back(Vector3i(vert2_index, vert23_index, vert1_index));
          new_triangle_vertex_indices.push_back(Vector3i(vert23_index, vert3_index, vert1_index));
        }
        else {
          int vert31_index = findEdgeVertexIndexOrInsertLambda(tri(2), tri(0));
          new_triangle_vertex_indices.push_back(Vector3i(vert3_index, vert31_index, vert2_index));
          new_triangle_vertex_indices.push_back(Vector3i(vert31_index, vert1_index, vert2_index));
        }
        break;
      }
      case SubdivisionStrategy::SUBDIVIDE_MIDPOINTS_4:
      {
        int vert12_index = findEdgeVertexIndexOrInsertLambda(tri(0), tri(1));
        int vert23_index = findEdgeVertexIndexOrInsertLambda(tri(1), tri(2));
        int vert31_index = findEdgeVertexIndexOrInsertLambda(tri(2), tri(0));

        // Insert the four new triangles resulting from the split of one triangle.
        new_triangle_vertex_indices.push_back(Vector3i(vert1_index, vert12_index, vert31_index));
        new_triangle_vertex_indices.push_back(Vector3i(vert12_index, vert2_index, vert23_index));
        new_triangle_vertex_indices.push_back(Vector3i(vert23_index, vert3_index, vert31_index));
        new_triangle_vertex_indices.push_back(Vector3i(vert12_index, vert23_index, vert31_index));
        break;
      }
      }
      ++subdivision_counter;
    }
    else {
      // Insert original triangle
      new_triangle_vertex_indices.push_back(Vector3i(vert1_index, vert2_index, vert3_index));
    }
  }

  triangle_vertex_indices_ = new_triangle_vertex_indices;

  if (hasNormals()) {
    BH_ASSERT(vertices_.size() == normals_.size());
  }

  if (hasTextureUVs()) {
    BH_ASSERT(vertices_.size() == texture_uvs_.size());
  }

  if (hasColors()) {
    BH_ASSERT(vertices_.size() == colors_.size());
  }

  return subdivision_counter;
}

template <typename FloatT, typename IndexT>
template <typename UnaryPredicate>
size_t TriangleMesh<FloatT, IndexT>::subdivideTrianglesUntil(const UnaryPredicate pred,
                                                             const SubdivisionStrategy sub_strategy) {
  // Divide each triangle and keep track of the new vertices between old vertices (connected by edges).
  using Edge = std::pair<IndexT, IndexT>;
  using EdgeToVertexIndexMap = std::unordered_map<Edge, int, bh::hash<Edge>>;
  using EdgeToVertexIterator = typename EdgeToVertexIndexMap::iterator;
  using FindReturnType = std::pair<EdgeToVertexIterator, bool>;
  EdgeToVertexIndexMap edgeToVertexIndexMap;

  std::vector<Vector3i> new_triangle_vertex_indices;
  std::deque<Vector3i> triangle_queue;
  size_t subdivision_counter = 0;
  std::copy(triangle_vertex_indices_.begin(), triangle_vertex_indices_.end(), std::front_inserter(triangle_queue));

  while (!triangle_queue.empty()) {
    const Vector3i tri = triangle_queue.front();
    triangle_queue.pop_front();
    const IndexT vert1_index = tri(0);
    const IndexT vert2_index = tri(1);
    const IndexT vert3_index = tri(2);
    const Vector3& vert1 = vertices_[vert1_index];
    const Vector3& vert2 = vertices_[vert2_index];
    const Vector3& vert3 = vertices_[vert3_index];
    const TriangleType triangle(vert1, vert2, vert3);
    if (pred(triangle)) {
      // Insert triangle
      new_triangle_vertex_indices.push_back(Vector3i(vert1_index, vert2_index, vert3_index));
    }
    else {
      // This lambda searches for an edge entry in the map. If there is no entry it inserts the corresponding vertex and
      // returns the new index. Edges are uniquely identified by ordering the indices.
      const auto findEdgeVertexIndexOrInsertLambda = [&] (IndexT index_a, IndexT index_b) -> IndexT {
        FindReturnType find_ret;
        if (index_a > index_b) {
          using std::swap;
          swap(index_a, index_b);
        }
        EdgeToVertexIterator vertex_iter;
        bool inserted;
        std::tie(vertex_iter, inserted) = edgeToVertexIndexMap.insert(std::make_pair(Edge(index_a, index_b), vertices_.size()));
        const IndexT found_vertex_index = vertex_iter->second;
        int vert_ab_index;
        if (inserted) {
          const Vector3& vert_a = vertices_[index_a];
          const Vector3& vert_b = vertices_[index_b];
          vert_ab_index = vertices_.size();
          vertices_.push_back((vert_a + vert_b) / 2);
          if (hasNormals()) {
            const Vector3& normal_a = normals_[index_a];
            const Vector3& normal_b = normals_[index_b];
            normals_.push_back((normal_a + normal_b).normalized());
          }
          if (hasTextureUVs()) {
            const Vector2& texture_uv_a = texture_uvs_[index_a];
            const Vector2& texture_uv_b = texture_uvs_[index_b];
            texture_uvs_.push_back((texture_uv_a + texture_uv_b) / 2);
          }
          if (hasColors()) {
            const ColorType& color_a = colors_[index_a];
            const ColorType& color_b = colors_[index_b];
            colors_.push_back((color_a + color_b) / 2);
          }
        }
        else {
          vert_ab_index = found_vertex_index;
        }
        return vert_ab_index;
      };
      switch (sub_strategy) {
        case SubdivisionStrategy::SUBDIVIDE_BASE_2:
        {
          const FloatType l1 = (vertices_[tri(1)] - vertices_[tri(0)]).squaredNorm();
          const FloatType l2 = (vertices_[tri(2)] - vertices_[tri(1)]).squaredNorm();
          const FloatType l3 = (vertices_[tri(0)] - vertices_[tri(2)]).squaredNorm();
          bool l1_is_max = false;
          bool l2_is_max = false;
          if (l1 >= l2) {
            if (l1 >= l3) {
              l1_is_max = true;
            }
          }
          else if (l2 >= l3) {
            l2_is_max = true;
          }
          if (l1_is_max) {
            int vert12_index = findEdgeVertexIndexOrInsertLambda(tri(0), tri(1));
            triangle_queue.push_back(Vector3i(vert1_index, vert12_index, vert3_index));
            triangle_queue.push_back(Vector3i(vert12_index, vert2_index, vert3_index));
          }
          else if (l2_is_max) {
            int vert23_index = findEdgeVertexIndexOrInsertLambda(tri(1), tri(2));
            triangle_queue.push_back(Vector3i(vert2_index, vert23_index, vert1_index));
            triangle_queue.push_back(Vector3i(vert23_index, vert3_index, vert1_index));
          }
          else {
            int vert31_index = findEdgeVertexIndexOrInsertLambda(tri(2), tri(0));
            triangle_queue.push_back(Vector3i(vert3_index, vert31_index, vert2_index));
            triangle_queue.push_back(Vector3i(vert31_index, vert1_index, vert2_index));
          }
          break;
        }
        case SubdivisionStrategy::SUBDIVIDE_MIDPOINTS_4:
        {
          int vert12_index = findEdgeVertexIndexOrInsertLambda(tri(0), tri(1));
          int vert23_index = findEdgeVertexIndexOrInsertLambda(tri(1), tri(2));
          int vert31_index = findEdgeVertexIndexOrInsertLambda(tri(2), tri(0));

          // Insert the four new triangles resulting from the split of one triangle.
          triangle_queue.push_back(Vector3i(vert1_index, vert12_index, vert31_index));
          triangle_queue.push_back(Vector3i(vert12_index, vert2_index, vert23_index));
          triangle_queue.push_back(Vector3i(vert23_index, vert3_index, vert31_index));
          triangle_queue.push_back(Vector3i(vert12_index, vert23_index, vert31_index));

          break;
        }
      }
      ++subdivision_counter;
    }
  }

  triangle_vertex_indices_ = new_triangle_vertex_indices;

  if (hasNormals()) {
    BH_ASSERT(vertices_.size() == normals_.size());
  }

  if (hasTextureUVs()) {
    BH_ASSERT(vertices_.size() == texture_uvs_.size());
  }

  if (hasColors()) {
    BH_ASSERT(vertices_.size() == colors_.size());
  }

  return subdivision_counter;
}

template <typename FloatT, typename IndexT>
auto TriangleMeshFactory<FloatT, IndexT>::createCanonicalTriangle() -> TriangleMeshType {
  TriangleMeshType tri_mesh;
  tri_mesh.triangle_vertex_indices_.resize(1);
  tri_mesh.vertices_.resize(3);
  tri_mesh.triangle_vertex_indices_(0) = 0;
  tri_mesh.triangle_vertex_indices_(1) = 1;
  tri_mesh.triangle_vertex_indices_(2) = 2;
  tri_mesh.vertices_.row(0)(0) = -1.0f;
  tri_mesh.vertices_.row(0)(1) =  0.0f;
  tri_mesh.vertices_.row(0)(2) =  0.0f;
  tri_mesh.vertices_.row(1)(0) = +1.0f;
  tri_mesh.vertices_.row(1)(1) =  0.0f;
  tri_mesh.vertices_.row(1)(2) =  0.0f;
  tri_mesh.vertices_.row(2)(0) =  0.0f;
  tri_mesh.vertices_.row(2)(1) =  1.0f;
  tri_mesh.vertices_.row(2)(2) =  0.0f;
  return tri_mesh;
}

template <typename FloatT, typename IndexT>
auto TriangleMeshFactory<FloatT, IndexT>::createIcosahedron(
    const FloatT radius, const bool generate_normals /*= true*/) -> TriangleMeshType {
  BH_ASSERT(radius > 0.0);
  TriangleMeshType tri_mesh;
  std::vector<Vector3i>& triangle_indices = tri_mesh.triangle_vertex_indices_;
  std::vector<Vector3>& vertices = tri_mesh.vertices_;

  // Golden ratio.
  FloatT phi = (1.0 + std::sqrt(5)) / 2.0;
  // Compute vertex coordinates for Icosahedron/
  FloatT b = 1.0 / std::sqrt(1.0 + phi * phi);
  FloatT a = phi * b;
  // Scale coordinates.
  a *= radius;
  b *= radius;

  // Generate vertices.
  vertices.push_back(Vector3( a,  b,  0));
  vertices.push_back(Vector3(-a,  b,  0));
  vertices.push_back(Vector3(-a, -b,  0));

  vertices.push_back(Vector3( a, -b,  0));
  vertices.push_back(Vector3( b,  0,  a));
  vertices.push_back(Vector3( b,  0, -a));

  vertices.push_back(Vector3(-b,  0, -a));
  vertices.push_back(Vector3(-b,  0,  a));
  vertices.push_back(Vector3( 0,  a,  b));

  vertices.push_back(Vector3( 0, -a,  b));
  vertices.push_back(Vector3( 0, -a, -b));
  vertices.push_back(Vector3( 0,  a, -b));

  // Generate stub normals
  if (generate_normals) {
    std::fill_n(std::back_inserter(tri_mesh.normals_), vertices.size(), Vector3::Zero());
  }

  // Generate corresponding triangles.
  enum icosahedron_index {ZA, ZB, ZC, ZD, YA, YB, YC, YD, XA, XB, XC, XD};
  triangle_indices.push_back(Vector3i(YA, XA, YD));
  triangle_indices.push_back(Vector3i(YA, YD, XB));
  triangle_indices.push_back(Vector3i(YB, YC, XD));
  triangle_indices.push_back(Vector3i(YB, XC, YC));
  triangle_indices.push_back(Vector3i(ZA, YA, ZD));
  triangle_indices.push_back(Vector3i(ZA, ZD, YB));
  triangle_indices.push_back(Vector3i(ZC, YD, ZB));
  triangle_indices.push_back(Vector3i(ZC, ZB, YC));
  triangle_indices.push_back(Vector3i(XA, ZA, XD));
  triangle_indices.push_back(Vector3i(XA, XD, ZB));
  triangle_indices.push_back(Vector3i(XB, XC, ZD));
  triangle_indices.push_back(Vector3i(XB, ZC, XC));
  triangle_indices.push_back(Vector3i(XA, YA, ZA));
  triangle_indices.push_back(Vector3i(XD, ZA, YB));
  triangle_indices.push_back(Vector3i(YA, XB, ZD));
  triangle_indices.push_back(Vector3i(YB, ZD, XC));
  triangle_indices.push_back(Vector3i(YD, XA, ZB));
  triangle_indices.push_back(Vector3i(YC, ZB, XD));
  triangle_indices.push_back(Vector3i(YD, ZC, XB));
  triangle_indices.push_back(Vector3i(YC, XC, ZC));

  return tri_mesh;
}

// TODO
//static TriangleMeshType createTriangularPrism(FloatT height, FloatT radius);

template <typename FloatT, typename IndexT>
auto TriangleMeshFactory<FloatT, IndexT>::createSphere(
    const FloatT radius,
    const size_t subdivisions,
    const SubdivisionStrategy sub_strategy) -> TriangleMeshType {
  TriangleMeshType tri_mesh = createIcosahedron(radius);
  for (size_t level = 0; level < subdivisions; ++level) {
    tri_mesh.subdivideTriangles(sub_strategy);
    // Normalize vertices
    for (Vector3& vertex : tri_mesh.vertices()) {
      vertex.normalize();
    }
  }
  return tri_mesh;
}

template <typename FloatT, typename IndexT>
auto TriangleMeshFactory<FloatT, IndexT>::createUnitSphere(
        const size_t subdivisions,
        const SubdivisionStrategy sub_strategy) -> TriangleMeshType {
  const FloatT radius = 1;
  return createSphere(radius, subdivisions, sub_strategy);
}

template <typename FloatT, typename IndexT>
typename TriangleMeshFactory<FloatT, IndexT>::CacheStorageUnitSphere
        TriangleMeshFactory<FloatT, IndexT>::unit_sphere_storage_ =
        TriangleMeshFactory<FloatT, IndexT>::CacheStorageUnitSphere(
    [](const std::pair<size_t, SubdivisionStrategy>& params) {
  const size_t subdivisions = params.first;
  const SubdivisionStrategy sub_strategy = params.second;
  return std::make_shared<TriangleMeshType>(createUnitSphere(subdivisions, sub_strategy));
});

template <typename FloatT, typename IndexT>
auto TriangleMeshFactory<FloatT, IndexT>::createSharedUnitSphere(
    const size_t subdivisions,
    const SubdivisionStrategy sub_strategy) -> std::shared_ptr<TriangleMeshType> {
  std::shared_ptr<TriangleMeshType> shared_mesh = unit_sphere_storage_.getInstance(
          std::make_pair(subdivisions, sub_strategy));
  return shared_mesh;
}

// TODO
//static TriangleMeshType createCylinder(FloatT height, FloatT radius, size_t subdivisions = 10);
//static TriangleMeshType createUnitRadiusCylinder(FloatT cylinder_length, size_t subdivisions = 10);

template <typename FloatT, typename IndexT>
template <typename TriIterator>
auto TriangleMeshFactory<FloatT, IndexT>::createFromTriangles(
    const TriIterator first, const TriIterator last) -> TriangleMeshType {
  TriangleMeshType mesh;
  mesh.triangleVertexIndices().resize(last - first);
  std::unordered_map<Vector3, size_t> vertex_index_map;
  const auto get_vertex_index_lambda = [&] (const Vector3& v) {
    const auto it = vertex_index_map.find(v);
    size_t idx;
    if (it == vertex_index_map.end()) {
      idx = mesh.m_Vertices.size();
      mesh.vertices().push_back(v);
      vertex_index_map.emplace(v, idx);
    }
    else {
      idx = vertex_index_map.at(v);
    }
    return idx;
  };
  mesh.triangle_vertex_indices_.reserve(last - first);
  for (TriIterator it = first; it != last; ++it) {
    const TriangleType& triangle = *it;
    const IndexT idx1 = get_vertex_index_lambda(triangle.v1());
    const IndexT idx2 = get_vertex_index_lambda(triangle.v2());
    const IndexT idx3 = get_vertex_index_lambda(triangle.v3());
    Vector3i tri_indices(idx1, idx2, idx3);
    mesh.triangle_vertex_indices_.push_back(std::move(tri_indices));
  }
  return mesh;
}

}
