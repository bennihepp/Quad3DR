//==================================================
// triangle_mesh.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Aug 20, 2016
//==================================================

#include <quad_planner/rendering/triangle_mesh.h>
#include <quad_planner/rendering/utilities.h>
#define _USE_MATH_DEFINES
#include <cmath>
#include <array>
#include <iostream>
#include <map>
#include <set>


namespace quad_planner
{
namespace rendering
{

TriangleMeshVectorStorage::TriangleMeshVectorStorage()
{
}

TriangleMeshVectorStorage::TriangleMeshVectorStorage(const TriangleMeshMatrixStorage& tri_mesh_mat)
{
  Utilities::convertMatrixToVector(tri_mesh_mat.vertices, &vertices);
  Utilities::convertMatrixToVector(tri_mesh_mat.normals, &normals);
  Utilities::convertMatrixToVector(tri_mesh_mat.triangles, &triangles);
}

int TriangleMeshVectorStorage::find_or_add_vertex(const Eigen::Vector3d& vertex, const Eigen::Vector3d& normal)
{
  for (auto it = vertices.cbegin(); it != vertices.cend(); ++it) {
    if (*it == vertex && normals[it - vertices.begin()] == normal) {
      return it - vertices.cbegin();
    }
  }
  vertices.push_back(vertex);
  normals.push_back(normal);
  return vertices.size() - 1;
}

int TriangleMeshVectorStorage::find_or_add_triangle(int vertexId1, int vertexId2, int vertexId3)
{
  Eigen::Vector3i tri1(vertexId1, vertexId2, vertexId3);
  Eigen::Vector3i tri2(vertexId2, vertexId3, vertexId1);
  Eigen::Vector3i tri3(vertexId3, vertexId1, vertexId2);
  for (auto it = triangles.cbegin(); it != triangles.cend(); ++it) {
    if (*it == tri1 || *it == tri2 || *it == tri3) {
      return it - triangles.cbegin();
    }
  }
  triangles.push_back(tri1);
  return triangles.size() - 1;
}

int TriangleMeshVectorStorage::find_or_add_triangle(const Eigen::Vector3d& vec1, const Eigen::Vector3d& vec2, const Eigen::Vector3d& vec3)
{
  int a = find_or_add_vertex(vec1, Eigen::Vector3d());
  int b = find_or_add_vertex(vec2, Eigen::Vector3d());
  int c = find_or_add_vertex(vec3, Eigen::Vector3d());
  return find_or_add_triangle(a, b, c);
}

void TriangleMeshVectorStorage::compute_averaged_normals(const std::vector<Eigen::Vector3i>& triangles, const std::vector<Eigen::Vector3d>& vertices, std::vector<Eigen::Vector3d>* normals_ptr)
{
  std::vector<Eigen::Vector3d>& normals = *normals_ptr;
  normals.resize(vertices.size());

  // Compute triangle normals.
  std::vector<Eigen::Vector3d> triangle_normals;
  for (auto it = triangles.cbegin(); it != triangles.cend(); ++it) {
    Eigen::Vector3d a = vertices[(*it)[1]] - vertices[(*it)[0]];
    Eigen::Vector3d b = vertices[(*it)[2]] - vertices[(*it)[1]];
    Eigen::Vector3d triangle_normal = a.cross(b).normalized();
    triangle_normals.push_back(triangle_normal);
  }

  // Compute mapping from vertex to connected triangles.
  std::map<int, std::set<int>> vertex_to_triangle_map;
  for (auto it = triangles.cbegin(); it != triangles.cend(); ++it) {
    int triangle_id = it - triangles.cbegin();
    for (int j = 0; j < 3; ++j) {
      int vertex_id = (*it)[j];
      auto map_it = vertex_to_triangle_map.find(vertex_id);
      if (map_it != vertex_to_triangle_map.cend()) {
        map_it->second.insert(triangle_id);
      }
      else {
        vertex_to_triangle_map.insert({vertex_id, std::set<int>({triangle_id})});
      }
    }
  }

  // Compute average normal for each vertex.
  for (auto it = vertices.cbegin(); it != vertices.cend(); ++it) {
    int vertex_id = it - vertices.cbegin();
    Eigen::Vector3d vertex_normal;
    vertex_normal.setZero();
    for (int triangle_id : vertex_to_triangle_map[it - vertices.cbegin()]) {
      vertex_normal += triangle_normals[triangle_id];
    }
    vertex_normal.normalize();
    normals[vertex_id] = vertex_normal;
  }
}

std::vector<Eigen::Vector3d> TriangleMeshVectorStorage::compute_averaged_normals(const std::vector<Eigen::Vector3i>& triangles, const std::vector<Eigen::Vector3d>& vertices)
{
  std::vector<Eigen::Vector3d> normals;
  compute_averaged_normals(triangles, vertices, &normals);
  return normals;
}

void TriangleMeshVectorStorage::compute_averaged_normals()
{
  compute_averaged_normals(triangles, vertices, &normals);
}

TriangleMeshMatrixStorage::TriangleMeshMatrixStorage()
{
}

TriangleMeshMatrixStorage::TriangleMeshMatrixStorage(const TriangleMeshVectorStorage& tri_mesh_vec)
{
  Utilities::convertVectorToMatrix(tri_mesh_vec.triangles, &triangles);
  Utilities::convertVectorToMatrix(tri_mesh_vec.vertices, &vertices);
  Utilities::convertVectorToMatrix(tri_mesh_vec.normals, &normals);
}


std::map<int, std::weak_ptr<TriangleMeshStorageOgl>> TriangleMeshFactory::unit_sphere_map_;
std::map<std::pair<double, int>, std::weak_ptr<TriangleMeshStorageOgl>> TriangleMeshFactory::unit_radius_cylinder_map_;

TriangleMeshVectorStorage TriangleMeshFactory::createTriangleVec()
{
  return TriangleMeshVectorStorage(createTriangleMat());
}

TriangleMeshMatrixStorage TriangleMeshFactory::createTriangleMat()
{
  TriangleMeshMatrixStorage tri_mesh_mat;
  tri_mesh_mat.triangles.resize(1, 3);
  tri_mesh_mat.vertices.resize(3, 3);
  tri_mesh_mat.triangles(0) = 0;
  tri_mesh_mat.triangles(1) = 1;
  tri_mesh_mat.triangles(2) = 2;
  tri_mesh_mat.vertices.row(0)(0) = -1.0f;
  tri_mesh_mat.vertices.row(0)(1) =  0.0f;
  tri_mesh_mat.vertices.row(0)(2) =  0.0f;
  tri_mesh_mat.vertices.row(1)(0) = +1.0f;
  tri_mesh_mat.vertices.row(1)(1) =  0.0f;
  tri_mesh_mat.vertices.row(1)(2) =  0.0f;
  tri_mesh_mat.vertices.row(2)(0) =  0.0f;
  tri_mesh_mat.vertices.row(2)(1) =  1.0f;
  tri_mesh_mat.vertices.row(2)(2) =  0.0f;
  return tri_mesh_mat;
}

std::shared_ptr<TriangleMeshStorageOgl> TriangleMeshFactory::createTriangleOgl()
{
  return std::make_shared<TriangleMeshStorageOgl>(createTriangleMat());
}

TriangleMeshVectorStorage TriangleMeshFactory::createIcosahedronVec(double radius)
{
  assert(radius > 0.0);
  TriangleMeshVectorStorage tri_mesh_vec;
  std::vector<Eigen::Vector3i>& triangle_vec = tri_mesh_vec.triangles;
  std::vector<Eigen::Vector3d>& vertex_vec = tri_mesh_vec.vertices;
  std::vector<Eigen::Vector3d>& normal_vec = tri_mesh_vec.normals;

  // Golden ratio.
  double phi = (1.0 + std::sqrt(5)) / 2.0;
  // Compute vertex coordinates for Icosahedron/
  double b = 1.0 / std::sqrt(1.0 + phi * phi);
  double a = phi * b;
  // Scale coordinates.
  a *= radius;
  b *= radius;

  // Generate vertices.
  vertex_vec.push_back(Eigen::Vector3d( a,  b,  0));
  vertex_vec.push_back(Eigen::Vector3d(-a,  b,  0));
  vertex_vec.push_back(Eigen::Vector3d(-a, -b,  0));

  vertex_vec.push_back(Eigen::Vector3d( a, -b,  0));
  vertex_vec.push_back(Eigen::Vector3d( b,  0,  a));
  vertex_vec.push_back(Eigen::Vector3d( b,  0, -a));

  vertex_vec.push_back(Eigen::Vector3d(-b,  0, -a));
  vertex_vec.push_back(Eigen::Vector3d(-b,  0,  a));
  vertex_vec.push_back(Eigen::Vector3d( 0,  a,  b));

  vertex_vec.push_back(Eigen::Vector3d( 0, -a,  b));
  vertex_vec.push_back(Eigen::Vector3d( 0, -a, -b));
  vertex_vec.push_back(Eigen::Vector3d( 0,  a, -b));

  // Generate stub normals
  for (int i = 0; i < vertex_vec.size(); ++i) {
    normal_vec.push_back(Eigen::Vector3d(0, 0, 0));
  }

  // Generate corresponding triangles.
  enum icosahedron_index {ZA, ZB, ZC, ZD, YA, YB, YC, YD, XA, XB, XC, XD};
  triangle_vec.push_back(Eigen::Vector3i(YA, XA, YD));
  triangle_vec.push_back(Eigen::Vector3i(YA, YD, XB));
  triangle_vec.push_back(Eigen::Vector3i(YB, YC, XD));
  triangle_vec.push_back(Eigen::Vector3i(YB, XC, YC));
  triangle_vec.push_back(Eigen::Vector3i(ZA, YA, ZD));
  triangle_vec.push_back(Eigen::Vector3i(ZA, ZD, YB));
  triangle_vec.push_back(Eigen::Vector3i(ZC, YD, ZB));
  triangle_vec.push_back(Eigen::Vector3i(ZC, ZB, YC));
  triangle_vec.push_back(Eigen::Vector3i(XA, ZA, XD));
  triangle_vec.push_back(Eigen::Vector3i(XA, XD, ZB));
  triangle_vec.push_back(Eigen::Vector3i(XB, XC, ZD));
  triangle_vec.push_back(Eigen::Vector3i(XB, ZC, XC));
  triangle_vec.push_back(Eigen::Vector3i(XA, YA, ZA));
  triangle_vec.push_back(Eigen::Vector3i(XD, ZA, YB));
  triangle_vec.push_back(Eigen::Vector3i(YA, XB, ZD));
  triangle_vec.push_back(Eigen::Vector3i(YB, ZD, XC));
  triangle_vec.push_back(Eigen::Vector3i(YD, XA, ZB));
  triangle_vec.push_back(Eigen::Vector3i(YC, ZB, XD));
  triangle_vec.push_back(Eigen::Vector3i(YD, ZC, XB));
  triangle_vec.push_back(Eigen::Vector3i(YC, XC, ZC));

  return tri_mesh_vec;
}

TriangleMeshMatrixStorage TriangleMeshFactory::createIcosahedronMat(double radius)
{
  TriangleMeshVectorStorage tri_mesh_vec = createIcosahedronVec(radius);
  tri_mesh_vec.compute_averaged_normals();
  return TriangleMeshMatrixStorage(tri_mesh_vec);
}

std::shared_ptr<TriangleMeshStorageOgl> TriangleMeshFactory::createIcosahedronOgl(double radius)
{
  return std::make_shared<TriangleMeshStorageOgl>(createIcosahedronMat(radius));
}

TriangleMeshVectorStorage TriangleMeshFactory::createTriangularPrismVec(double height, double radius)
{
  assert(height > 0.0);
  assert(radius > 0.0);

  const double X0 = std::cos(0 * M_PI / 180.0);
  const double Y0 = std::sin(0 * M_PI / 180.0);
  const double X1 = std::cos(120 * M_PI / 180.0);
  const double Y1 = std::sin(120 * M_PI / 180.0);
  const double X2 = std::cos(240 * M_PI / 180.0);
  const double Y2 = std::sin(240 * M_PI / 180.0);

  const std::array<double, 4> x_array = { X0, X1, X2, X0};
  const std::array<double, 4> y_array = { Y0, Y1, Y2, Y0 };

  TriangleMeshVectorStorage tri_mesh_vec;
  std::vector<Eigen::Vector3i>& triangle_vec = tri_mesh_vec.triangles;
  std::vector<Eigen::Vector3d>& vertex_vec = tri_mesh_vec.vertices;

  double ratio_height_radius = height / radius;
  int num_of_height_segments = std::ceil(ratio_height_radius);

  // Generate vertices for sides.
  for (int i = 0; i < num_of_height_segments; ++i) {
    double z0 = i * height / num_of_height_segments - height / 2.0;
    double z1 = (i + 1) * height / num_of_height_segments  - height / 2.0;

    // Generate two triangles for each side of the triangle to make up the extruded base.
    for (int j = 0; j < 3; ++j) {
      double x0 = x_array[j];
      double y0 = y_array[j];
      double x1 = x_array[j + 1];
      double y1 = y_array[j + 1];

      tri_mesh_vec.find_or_add_triangle(
          Eigen::Vector3d(x0, y0, z0),
          Eigen::Vector3d(x1, y1, z0),
          Eigen::Vector3d(x0, y0, z1)
      );
      tri_mesh_vec.find_or_add_triangle(
          Eigen::Vector3d(x1, y1, z0),
          Eigen::Vector3d(x1, y1, z1),
          Eigen::Vector3d(x0, y0, z1)
      );
    }

    // Generate vertices for cap at bottom.
    if (i == 0) {
      double z = z0;
      std::array<Eigen::Vector3d, 3> vertices;
      for (int j = 0; j < 3; ++j) {
        double x = x_array[2 - j];
        double y = y_array[2 - j];
        vertices[j] = Eigen::Vector3d(x, y, z);
      }
      tri_mesh_vec.find_or_add_triangle(vertices[0], vertices[1], vertices[2]);
    }
    else if (i == num_of_height_segments - 1) {
      // Generate vertices for cap at top.
      double z = z1;
      std::array<Eigen::Vector3d, 3> vertices;
      for (int j = 0; j < 3; ++j) {
        double x = x_array[j];
        double y = y_array[j];
        vertices[j] = Eigen::Vector3d(x, y, z);
      }
      tri_mesh_vec.find_or_add_triangle(vertices[0], vertices[1], vertices[2]);
    }
  }

  return tri_mesh_vec;
}

TriangleMeshMatrixStorage TriangleMeshFactory::createTriangularPrismMat(double height, double radius)
{
  TriangleMeshVectorStorage tri_mesh_vec = createTriangularPrismVec(height, radius);
  return TriangleMeshMatrixStorage(tri_mesh_vec);
}

std::shared_ptr<TriangleMeshStorageOgl> TriangleMeshFactory::createTriangularPrismOgl(double height, double radius)
{
  return std::make_shared<TriangleMeshStorageOgl>(createTriangularPrismMat(height, radius));
}

TriangleMeshVectorStorage TriangleMeshFactory::createSphereVec(double radius, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = createSphereMat(radius, subdivisions);
  return TriangleMeshVectorStorage(tri_mesh_mat);
}

TriangleMeshMatrixStorage TriangleMeshFactory::createSphereMat(double radius, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshFactory::createIcosahedronMat(radius);
  for (int level = 0; level < subdivisions; ++level)
  {
    TriangleMeshFactory::subdivideMesh(&tri_mesh_mat);
    // Normalize vertices
    for (int i = 0; i < tri_mesh_mat.vertices.cols(); ++i)
    {
      Eigen::Vector3d vert = tri_mesh_mat.vertices.col(i);
      tri_mesh_mat.vertices.col(i) = vert / vert.norm();
    }
  }
  return tri_mesh_mat;
}

std::shared_ptr<TriangleMeshStorageOgl> TriangleMeshFactory::createSphereOgl(double radius, int subdivisions)
{
  return std::make_shared<TriangleMeshStorageOgl>(createSphereMat(radius, subdivisions));
}

std::shared_ptr<TriangleMeshStorageOgl> TriangleMeshFactory::createUnitSphereOgl(int subdivisions)
{
  auto it = unit_sphere_map_.find(subdivisions);
  if (it != unit_sphere_map_.cend())
  {
    std::weak_ptr<TriangleMeshStorageOgl> unit_sphere_weak_ptr = it->second;
    auto unit_sphere_shared_ptr = unit_sphere_weak_ptr.lock();
    if (unit_sphere_shared_ptr)
    {
      return unit_sphere_shared_ptr;
    }
  }
  std::shared_ptr<TriangleMeshStorageOgl> unit_sphere_shared_ptr = createSphereOgl(1.0, subdivisions);
  std::weak_ptr<TriangleMeshStorageOgl> unit_sphere_weak_ptr(unit_sphere_shared_ptr);
  unit_sphere_map_.insert({subdivisions, unit_sphere_weak_ptr});
  return unit_sphere_shared_ptr;
}

TriangleMeshVectorStorage TriangleMeshFactory::createCylinderVec(double height, double radius, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = createCylinderMat(height, radius, subdivisions);
  return TriangleMeshVectorStorage(tri_mesh_mat);
}

TriangleMeshMatrixStorage TriangleMeshFactory::createCylinderMat(double height, double radius, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshFactory::createTriangularPrismMat(height, radius);
  for (int level = 0; level < subdivisions; ++level)
  {
    TriangleMeshFactory::subdivideMesh(&tri_mesh_mat);
    // Normalize vertices in x-y dimension
    for (int i = 0; i < tri_mesh_mat.vertices.cols(); ++i)
    {
      Eigen::Vector2d vert = tri_mesh_mat.vertices.col(i).segment(0, 2);
      tri_mesh_mat.vertices.col(i).segment(0, 2) = vert / vert.norm();
    }
  }
  return tri_mesh_mat;
}

std::shared_ptr<TriangleMeshStorageOgl> TriangleMeshFactory::createCylinderOgl(double height, double radius, int subdivisions)
{
  return std::make_shared<TriangleMeshStorageOgl>(createCylinderMat(height, radius, subdivisions));
}

std::shared_ptr<TriangleMeshStorageOgl> TriangleMeshFactory::createUnitRadiusCylinderOgl(double height, int subdivisions)
{
  auto key = std::make_pair(height, subdivisions);
  auto it = unit_radius_cylinder_map_.find(key);
  if (it != unit_radius_cylinder_map_.cend())
  {
    std::weak_ptr<TriangleMeshStorageOgl> unit_sphere_weak_ptr = it->second;
    auto unit_sphere_shared_ptr = unit_sphere_weak_ptr.lock();
    if (unit_sphere_shared_ptr)
    {
      return unit_sphere_shared_ptr;
    }
  }
  std::shared_ptr<TriangleMeshStorageOgl> unit_radius_shared_ptr = createCylinderOgl(height, 1.0, subdivisions);
  std::weak_ptr<TriangleMeshStorageOgl> unit_radius_weak_ptr(unit_radius_shared_ptr);
  unit_radius_cylinder_map_.insert({key, unit_radius_weak_ptr});
  return unit_radius_shared_ptr;
}


void TriangleMeshFactory::subdivideMesh(TriangleMeshMatrixStorage *tri_mesh_mat)
{
  std::vector<Eigen::Vector3i> triangles;
  std::vector<Eigen::Vector3d> vertices;
  std::vector<Eigen::Vector3d> normals;
  // Copy old vertices.
  for (int i = 0; i < tri_mesh_mat->vertices.cols(); ++i)
  {
    vertices.push_back(tri_mesh_mat->vertices.col(i));
    normals.push_back(tri_mesh_mat->normals.col(i));
  }

  // Divide each triangle and keep track of the new vertices between old vertices (connected by edges).
  using Edge = std::pair<int, int>;
  using EdgeToVertexIndexMap = std::map<Edge, int>;
  using FindReturnType = std::pair<EdgeToVertexIndexMap::iterator, bool>;
  EdgeToVertexIndexMap edgeToVertexIndexMap;
  for (int i = 0; i < tri_mesh_mat->triangles.cols(); ++i)
  {
    const Eigen::Vector3i& tri = tri_mesh_mat->triangles.col(i);
    int vert1_index = tri(0);
    int vert2_index = tri(1);
    int vert3_index = tri(2);
    const Eigen::Vector3d& vert1 = tri_mesh_mat->vertices.col(vert1_index);
    const Eigen::Vector3d& vert2 = tri_mesh_mat->vertices.col(vert2_index);
    const Eigen::Vector3d& vert3 = tri_mesh_mat->vertices.col(vert3_index);

    // This lambda searches for an edge entry in the map. If there is no entry it inserts the corresponding vertex and
    // returns the new index. Edges are uniquely identified by ordering the indices.
    auto findEdgeVertexIndexOrInsertLambda = [&vertices, &normals, &edgeToVertexIndexMap, tri_mesh_mat](int index_a, int index_b) -> int
    {
      FindReturnType find_ret;
      bool inserted;
      int found_vertex_index;
      if (index_a > index_b)
      {
        std::swap(index_a, index_b);
      }
      find_ret = edgeToVertexIndexMap.insert(std::make_pair(Edge(index_a, index_b), vertices.size()));
      inserted = find_ret.second;
      found_vertex_index = find_ret.first->second;
      int vert_ab_index;
      if (inserted)
      {
        const Eigen::Vector3d& vert_a = tri_mesh_mat->vertices.col(index_a);
        const Eigen::Vector3d& vert_b = tri_mesh_mat->vertices.col(index_b);
        const Eigen::Vector3d& normal_a = tri_mesh_mat->normals.col(index_a);
        const Eigen::Vector3d& normal_b = tri_mesh_mat->normals.col(index_b);
        vert_ab_index = vertices.size();
        Eigen::Vector3d vert_ab = (vert_a + vert_b) / 2;
        vertices.push_back(vert_ab);
        normals.push_back((normal_a + normal_b).normalized());
      }
      else
      {
        vert_ab_index = found_vertex_index;
      }
      return vert_ab_index;
    };

    int vert12_index = findEdgeVertexIndexOrInsertLambda(tri(0), tri(1));
    int vert23_index = findEdgeVertexIndexOrInsertLambda(tri(1), tri(2));
    int vert31_index = findEdgeVertexIndexOrInsertLambda(tri(2), tri(0));

    // Insert the four new triangles resulting from the split of one triangle.
    triangles.push_back(Eigen::Vector3i(vert1_index, vert12_index, vert31_index));
    triangles.push_back(Eigen::Vector3i(vert12_index, vert2_index, vert23_index));
    triangles.push_back(Eigen::Vector3i(vert23_index, vert3_index, vert31_index));
    triangles.push_back(Eigen::Vector3i(vert12_index, vert23_index, vert31_index));
  }

  Utilities::convertVectorToMatrix(triangles, &tri_mesh_mat->triangles);
  Utilities::convertVectorToMatrix(vertices, &tri_mesh_mat->vertices);
  Utilities::convertVectorToMatrix(normals, &tri_mesh_mat->normals);
}



TriangleMeshStorageOgl::TriangleMeshStorageOgl(const TriangleMeshMatrixStorage& tri_mesh_mat)
{
  init(tri_mesh_mat.triangles, tri_mesh_mat.vertices, tri_mesh_mat.normals);
}

TriangleMeshStorageOgl::TriangleMeshStorageOgl(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices)
{
  init(triangles, vertices);
}

TriangleMeshStorageOgl::TriangleMeshStorageOgl(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& normals)
{
  init(triangles, vertices, normals);
}

TriangleMeshStorageOgl::~TriangleMeshStorageOgl()
{
  glDeleteBuffers(1, &index_buffer_id_);
  glDeleteBuffers(1, &vertex_buffer_id_);
  glDeleteBuffers(1, &normal_buffer_id_);
}

void TriangleMeshStorageOgl::init(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices)
{
  std::vector<Eigen::Vector3i> triangles_vector;
  std::vector<Eigen::Vector3d> vertices_vector;
  Utilities::convertMatrixToVector(triangles, &triangles_vector);
  Utilities::convertMatrixToVector(vertices, &vertices_vector);
  std::vector<Eigen::Vector3d> normals_vector =
      TriangleMeshVectorStorage::compute_averaged_normals(triangles_vector, vertices_vector);
  Eigen::MatrixXd normals;
  Utilities::convertVectorToMatrix(normals_vector, &normals);
  init(triangles, vertices, normals);
}

void TriangleMeshStorageOgl::init(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& normals)
{
  assert(normals.size() == 0 || normals.size() == vertices.size());
  assert(triangles.rows() == 3);
  assert(vertices.rows() == 3);

  //  std::cout << "Mesh has " << (triangles.rows()) << " triangles and "
  //            << (vertices.size()) << " vertices and "
  //            << (colors.size()) << " colors and "
  //            << (vertices.size() * 3) << " values" << std::endl;
  //    std::cout << triangles << std::endl;
  //    std::cout << vertices << std::endl;
  //    for (int i=0; i < vertices.size(); ++i)
  //    {
  //      std::cout << "i: " << vertices.data()[i] << std::endl;
  //    }

  // Vertex attribute buffer
  glGenBuffers(1, &vertex_buffer_id_);
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(GLdouble), vertices.data(), GL_STATIC_DRAW);

  // Normals buffer
  glGenBuffers(1, &normal_buffer_id_);
  glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(GLdouble), normals.data(), GL_STATIC_DRAW);

  // Index buffer
  glGenBuffers(1, &index_buffer_id_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_id_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, triangles.size() * sizeof(GLuint), triangles.data(), GL_STATIC_DRAW);
  num_of_elements_ = triangles.size();
}

void TriangleMeshStorageOgl::bindVertexBuffer()
{
  glBindBuffer(GL_ARRAY_BUFFER, vertex_buffer_id_);
}

void TriangleMeshStorageOgl::bindNormalBuffer()
{
  glBindBuffer(GL_ARRAY_BUFFER, normal_buffer_id_);
}

void TriangleMeshStorageOgl::bindIndexBuffer()
{
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, index_buffer_id_);
}

GLsizei TriangleMeshStorageOgl::getNumOfElements() const
{
  return num_of_elements_;
}


TriangleMesh::TriangleMesh(const std::shared_ptr<TriangleMeshStorageOgl>& tri_mesh_ogl_ptr)
: tri_mesh_ogl_ptr_(tri_mesh_ogl_ptr)
{
  initVAO();
  enableVertexVBOs();
  finishInit();
}

TriangleMesh::TriangleMesh(const TriangleMeshMatrixStorage& tri_mesh_mat)
{
  initVAO();
  tri_mesh_ogl_ptr_ = std::make_shared<TriangleMeshStorageOgl>(tri_mesh_mat.triangles, tri_mesh_mat.vertices, tri_mesh_mat.normals);
  enableVertexVBOs();
  finishInit();
}

TriangleMesh::TriangleMesh(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices)
{
  initVAO();
  tri_mesh_ogl_ptr_ = std::make_shared<TriangleMeshStorageOgl>(triangles, vertices);
  enableVertexVBOs();
  finishInit();
}

TriangleMesh::TriangleMesh(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& normals)
{
  initVAO();
  tri_mesh_ogl_ptr_ = std::make_shared<TriangleMeshStorageOgl>(triangles, vertices, normals);
  enableVertexVBOs();
  finishInit();
}

TriangleMesh::~TriangleMesh()
{
  glDeleteVertexArrays(1, &vertex_array_id_);
}

void TriangleMesh::initVAO()
{
  // Setup vertex arrays
  glGenVertexArrays(1, &vertex_array_id_);
  glBindVertexArray(vertex_array_id_);
}

void TriangleMesh::enableVertexVBOs()
{
  // Vertex attribute buffer
  tri_mesh_ogl_ptr_->bindVertexBuffer();
  glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(0);
  tri_mesh_ogl_ptr_->bindNormalBuffer();
  glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(1);

  // Index buffer
  tri_mesh_ogl_ptr_->bindIndexBuffer();
}

void TriangleMesh::finishInit()
{
  glBindVertexArray(0);
}

void TriangleMesh::render() const
{
  glBindVertexArray(vertex_array_id_);
  glDrawElements(GL_TRIANGLES, tri_mesh_ogl_ptr_->getNumOfElements(), GL_UNSIGNED_INT, nullptr);
}

std::shared_ptr<TriangleMesh> TriangleMesh::createTriangle()
{
  Eigen::MatrixXi triangles(1, 3);
  Eigen::MatrixXd vertices(3, 3);
  triangles(0) = 0;
  triangles(1) = 1;
  triangles(2) = 2;
  vertices.row(0)(0) = -1.0f;
  vertices.row(0)(1) =  0.0f;
  vertices.row(0)(2) =  0.0f;
  vertices.row(1)(0) = +1.0f;
  vertices.row(1)(1) =  0.0f;
  vertices.row(1)(2) =  0.0f;
  vertices.row(2)(0) =  0.0f;
  vertices.row(2)(1) =  1.0f;
  vertices.row(2)(2) =  0.0f;
  return std::make_shared<TriangleMesh>(triangles.transpose(), vertices.transpose());
}

std::shared_ptr<TriangleMesh> TriangleMesh::createIcosahedron(double radius)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshFactory::createIcosahedronMat(radius);
  return std::make_shared<TriangleMesh>(tri_mesh_mat);
}

std::shared_ptr<TriangleMesh> TriangleMesh::createSphere(double radius, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshFactory::createSphereMat(radius, subdivisions);
  return std::make_shared<TriangleMesh>(tri_mesh_mat);
}

std::shared_ptr<TriangleMesh> TriangleMesh::createUnitSphere(int subdivisions)
{
  std::shared_ptr<TriangleMeshStorageOgl> tri_mesh_ogl = TriangleMeshFactory::createUnitSphereOgl(subdivisions);
  return std::make_shared<TriangleMesh>(tri_mesh_ogl);
}

std::shared_ptr<TriangleMesh> TriangleMesh::createUnitRadiusCylinder(double height, int subdivisions)
{
  std::shared_ptr<TriangleMeshStorageOgl> tri_mesh_ogl = TriangleMeshFactory::createUnitRadiusCylinderOgl(height, subdivisions);
  return std::make_shared<TriangleMesh>(tri_mesh_ogl);
}


ColorTriangleMesh::ColorTriangleMesh(const std::shared_ptr<TriangleMeshStorageOgl>& tri_mesh_ogl_ptr, const Eigen::MatrixXd& colors)
: tri_mesh_ogl_ptr_(tri_mesh_ogl_ptr)
{
  initVAO();
  initColorVBO(colors);
  enableVertexVBOs();
  finishInit();
}

ColorTriangleMesh::ColorTriangleMesh(const TriangleMeshMatrixStorage& tri_mesh_mat, const Eigen::MatrixXd& colors)
{
  initVAO();
  tri_mesh_ogl_ptr_ = std::make_shared<TriangleMeshStorageOgl>(tri_mesh_mat.triangles, tri_mesh_mat.vertices);
  enableVertexVBOs();
  initColorVBO(colors);
  finishInit();
}

ColorTriangleMesh::ColorTriangleMesh(const Eigen::MatrixXi& triangles, const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& colors)
{
  initVAO();
  tri_mesh_ogl_ptr_ = std::make_shared<TriangleMeshStorageOgl>(triangles, vertices);
  enableVertexVBOs();
  initColorVBO(colors);
  finishInit();
}

ColorTriangleMesh::~ColorTriangleMesh()
{
  glDeleteBuffers(1, &color_buffer_id_);
  glDeleteVertexArrays(1, &vertex_array_id_);
}

void ColorTriangleMesh::initVAO()
{
  // Setup vertex arrays
  glGenVertexArrays(1, &vertex_array_id_);
  glBindVertexArray(vertex_array_id_);
}

void ColorTriangleMesh::enableVertexVBOs()
{
  // Vertex attribute buffer
  tri_mesh_ogl_ptr_->bindVertexBuffer();
  glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(0);
  tri_mesh_ogl_ptr_->bindNormalBuffer();
  glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(1);

  // Index buffer
  tri_mesh_ogl_ptr_->bindIndexBuffer();
}

void ColorTriangleMesh::initColorVBO(const Eigen::MatrixXd& colors)
{
  // Color attribute buffer
  glGenBuffers(1, &color_buffer_id_);
  glBindBuffer(GL_ARRAY_BUFFER, color_buffer_id_);
  glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(GLdouble), colors.data(), GL_STATIC_DRAW);
  glVertexAttribPointer(2, 3, GL_DOUBLE, GL_FALSE, 0, nullptr);
  glEnableVertexAttribArray(2);
}

void ColorTriangleMesh::finishInit()
{
  glBindVertexArray(0);
}

void ColorTriangleMesh::render() const
{
  glBindVertexArray(vertex_array_id_);
  glDrawElements(GL_TRIANGLES, tri_mesh_ogl_ptr_->getNumOfElements(), GL_UNSIGNED_INT, nullptr);
}

std::shared_ptr<ColorTriangleMesh> ColorTriangleMesh::createTriangle(const Eigen::Vector3d& color)
{
  Eigen::MatrixXi triangles(1, 3);
  Eigen::MatrixXd vertices(3, 3);
  Eigen::MatrixXd colors(3, 3);
  triangles(0) = 0;
  triangles(1) = 1;
  triangles(2) = 2;
  vertices.row(0)(0) = -1.0f;
  vertices.row(0)(1) =  0.0f;
  vertices.row(0)(2) =  0.0f;
  vertices.row(1)(0) = +1.0f;
  vertices.row(1)(1) =  0.0f;
  vertices.row(1)(2) =  0.0f;
  vertices.row(2)(0) =  0.0f;
  vertices.row(2)(1) =  1.0f;
  vertices.row(2)(2) =  0.0f;
  colors.col(0) = color;
  colors.col(1) = color;
  colors.col(2) = color;
  return std::make_shared<ColorTriangleMesh>(triangles.transpose(), vertices.transpose(), colors.transpose());
}

std::shared_ptr<ColorTriangleMesh> ColorTriangleMesh::createIcosahedron(double radius, const Eigen::Vector3d& color)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshFactory::createIcosahedronMat(radius);
  Eigen::MatrixXd colors(3, tri_mesh_mat.vertices.cols());
  for (int i = 0; i < colors.cols(); ++i)
  {
    colors.col(i) = color;
  }
  return std::make_shared<ColorTriangleMesh>(tri_mesh_mat, colors);
}

std::shared_ptr<ColorTriangleMesh> ColorTriangleMesh::createSphere(double radius, const Eigen::Vector3d& color, int subdivisions)
{
  TriangleMeshMatrixStorage tri_mesh_mat = TriangleMeshFactory::createSphereMat(radius, subdivisions);
  Eigen::MatrixXd colors(3, tri_mesh_mat.vertices.cols());
  for (int i = 0; i < colors.cols(); ++i)
  {
    colors.col(i) = color;
  }
  return std::make_shared<ColorTriangleMesh>(tri_mesh_mat, colors);
}

std::shared_ptr<ColorTriangleMesh> ColorTriangleMesh::createUnitSphere(const Eigen::Vector3d& color, int subdivisions)
{
  std::shared_ptr<TriangleMeshStorageOgl> tri_mesh_ogl = TriangleMeshFactory::createUnitSphereOgl(subdivisions);
  Eigen::MatrixXd colors(3, tri_mesh_ogl->getNumOfElements());
  for (int i = 0; i < colors.cols(); ++i)
  {
    colors.col(i) = color;
  }
  return std::make_shared<ColorTriangleMesh>(tri_mesh_ogl, colors);
}

std::shared_ptr<ColorTriangleMesh> ColorTriangleMesh::createUnitRadiusCylinder(double height, const Eigen::Vector3d& color, int subdivisions)
{
  std::shared_ptr<TriangleMeshStorageOgl> tri_mesh_ogl = TriangleMeshFactory::createUnitRadiusCylinderOgl(height, subdivisions);
  Eigen::MatrixXd colors(3, tri_mesh_ogl->getNumOfElements());
  for (int i = 0; i < colors.cols(); ++i)
  {
    colors.col(i) = color;
  }
  return std::make_shared<ColorTriangleMesh>(tri_mesh_ogl, colors);
}

} /* namespace rendering */
} /* namespace quad_planner */
