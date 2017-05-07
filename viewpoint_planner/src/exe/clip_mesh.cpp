//==================================================
// clip_mesh.cpp
//
//  Copyright (c) 2016 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: Feb 20, 2017
//==================================================

#include <iostream>
#include <memory>
#include <csignal>

#include <bh/boost.h>
#include <boost/property_tree/json_parser.hpp>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <bh/common.h>
#include <bh/eigen.h>
#include <bh/utilities.h>
#include <bh/config_options.h>
#include <bh/eigen_options.h>
#include <bh/math/geometry.h>
#include <bh/gps.h>

#include <bh/mLib/mLib.h>

#include "../reconstruction/dense_reconstruction.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using FloatType = float;
USE_FIXED_EIGEN_TYPES(FloatType)

using BoundingBoxType = bh::BoundingBox3D<FloatType>;
using MeshType = ml::MeshData<FloatType>;
using MeshIOType = ml::MeshIO<FloatType>;
using TriMeshType = ml::TriMesh<FloatType>;

class ClipAndTransformMeshCmdline {
public:

  class Options : public bh::ConfigOptions {
  public:
    static const string kPrefix;

    Options()
    : bh::ConfigOptions(kPrefix) {
      addOption<Vector3>("offset_vector", &offset_vector);
      addOption<FloatType>("clip_distance", &clip_distance);
      addOption<Vector3>("clip_bbox_min", &clip_bbox_min);
      addOption<Vector3>("clip_bbox_max", &clip_bbox_max);
      addOption<bool>("use_region_file", &use_region_file);
      addOption<string>("regions_json_filename", &regions_json_filename);
      addOption<string>("dense_reconstruction_path", &dense_reconstruction_path);
    }

    ~Options() override {}

    Vector3 offset_vector = Vector3(0, 0, 0);
    FloatType clip_distance = 0;
    Vector3 clip_bbox_min;
    Vector3 clip_bbox_max;
    bool use_region_file = false;
    string regions_json_filename;
    string dense_reconstruction_path;
  };

  static std::map<string, std::unique_ptr<bh::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<bh::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new Options())));
    return config_options;
  }

  ClipAndTransformMeshCmdline(const std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options,
      const string& in_mesh_filename, const string& out_mesh_filename)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    in_mesh_filename_(in_mesh_filename),
    out_mesh_filename_(out_mesh_filename) {}

  ~ClipAndTransformMeshCmdline() {
  }

  using GpsCoordinateType = reconstruction::SfmToGpsTransformation::GpsCoordinate;
  using GpsFloatType = typename GpsCoordinateType::FloatType;
  using GpsConverter = bh::GpsConverter<GpsFloatType>;
  using RegionType = bh::PolygonWithLowerAndUpperPlane<FloatType>;

  // TODO: Move into viewpoint_planner_common
  RegionType convertGpsRegionToEnuRegion(
          const GpsConverter& gps_converter,
          const boost::property_tree::ptree& pt) const {
    const bool verbose = true;
    const FloatType lower_altitude = pt.get<FloatType>("lower_altitude");
    const FloatType upper_altitude = pt.get<FloatType>("upper_altitude");
    std::vector<GpsCoordinateType> gps_coordinates;
    for (const boost::property_tree::ptree::value_type& v : pt.get_child("latlong_vertices")) {
      gps_coordinates.push_back(GpsCoordinateType(v.second.get<FloatType>("latitude"), v.second.get<FloatType>("longitude"), 0));
    }
    std::vector<RegionType::Vector2> vertices;
    for (const GpsCoordinateType& gps : gps_coordinates) {
      const Vector3 enu = gps_converter.convertGpsToEnu(gps).cast<FloatType>();
      if (verbose) {
        std::cout << "GPS: " << gps << ", ENU: " << enu.transpose() << std::endl;
      }
      vertices.emplace_back(enu(0), enu(1));

      // Test code for GPS conversion
//    GpsCoordinateType gps2 = gps;
//    BH_PRINT_VALUE(gps2);
//    BH_PRINT_VALUE(gps_converter.convertGpsToEcef(gps2));
//    BH_PRINT_VALUE(gps_converter.convertGpsToEnu(gps2));
//    gps2 = GpsCoordinateType(gps2.latitude(), gps2.longitude(), gps2.altitude() + 360);
//    BH_PRINT_VALUE(gps2);
//    BH_PRINT_VALUE(gps_converter.convertGpsToEcef(gps2));
//    BH_PRINT_VALUE(gps_converter.convertGpsToEnu(gps2));
//    Vector3 enu2 = enu;
//    BH_PRINT_VALUE(enu2);
//    BH_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
//    enu2(2) = 360;
//    BH_PRINT_VALUE(enu2);
//    BH_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
//    enu2(2) = 0;
//    BH_PRINT_VALUE(enu2);
//    BH_PRINT_VALUE(gps_converter.convertEnuToGps(enu2.cast<GpsFloatType>()));
    }
    if (verbose) {
      std::cout << "Lower altitude: " << lower_altitude << ", upper_altitude: " << upper_altitude << std::endl;
    }
    const RegionType region(vertices, lower_altitude, upper_altitude);
    return region;
  }

  template <typename Predicate>
  MeshType clipMesh(const MeshType& mesh, Predicate&& predicate) {
    MeshType clipped_mesh;
    std::unordered_map<size_t, size_t> old_to_new_vertex_indices;
    std::unordered_map<size_t, size_t> old_to_new_color_indices;
    std::unordered_map<size_t, size_t> old_to_new_normal_indices;
    std::unordered_map<size_t, size_t> old_to_new_textcoord_indices;
    for (std::size_t i = 0; i < mesh.m_Vertices.size(); ++i) {
      const ml::vec3<FloatType>& ml_vertex = mesh.m_Vertices[i];
      const Vector3 vertex(ml_vertex.x, ml_vertex.y, ml_vertex.z);
      if (std::forward<Predicate>(predicate)(vertex)) {
        old_to_new_vertex_indices.emplace(i, clipped_mesh.m_Vertices.size());
        clipped_mesh.m_Vertices.push_back(mesh.m_Vertices[i]);
        if (mesh.hasColors()) {
          old_to_new_color_indices.emplace(i, clipped_mesh.m_Colors.size());
          clipped_mesh.m_Colors.push_back(mesh.m_Colors[i]);
        }
        if (mesh.hasNormals()) {
          old_to_new_normal_indices.emplace(i, clipped_mesh.m_Normals.size());
          clipped_mesh.m_Normals.push_back(mesh.m_Normals[i]);
        }
        if (mesh.hasTexCoords()) {
          old_to_new_textcoord_indices.emplace(i, clipped_mesh.m_TextureCoords.size());
          clipped_mesh.m_TextureCoords.push_back(mesh.m_TextureCoords[i]);
        }
      }
    }
    for (std::size_t i = 0; i < mesh.m_FaceIndicesVertices.size(); ++i) {
      const MeshType::Indices::Face& vertex_face = mesh.m_FaceIndicesVertices[i];
      BH_ASSERT_STR(vertex_face.size() == 3, "Mesh faces need to have a valence of 3");
      size_t index1 = vertex_face[0];
      size_t index2 = vertex_face[1];
      size_t index3 = vertex_face[2];
      if (old_to_new_vertex_indices.count(index1) == 0
          || old_to_new_vertex_indices.count(index2) == 0
          || old_to_new_vertex_indices.count(index3) == 0) {
        continue;
      }
      MeshType::Indices::Face new_vertex_face;
      new_vertex_face.setSize(3);
      new_vertex_face.setPtr(new unsigned int[3]);
      new_vertex_face[0] = old_to_new_vertex_indices.at(index1);
      new_vertex_face[1] = old_to_new_vertex_indices.at(index2);
      new_vertex_face[2] = old_to_new_vertex_indices.at(index3);
      clipped_mesh.m_FaceIndicesVertices.push_back(new_vertex_face);
      if (mesh.hasColorIndices()) {
        const MeshType::Indices::Face& color_face = mesh.m_FaceIndicesColors[i];
        MeshType::Indices::Face new_color_face;
        new_color_face.setSize(3);
        new_color_face.setPtr(new unsigned int[3]);
        new_color_face[0] = old_to_new_color_indices.at(color_face[0]);
        new_color_face[1] = old_to_new_color_indices.at(color_face[1]);
        new_color_face[2] = old_to_new_color_indices.at(color_face[2]);
        clipped_mesh.m_FaceIndicesColors.push_back(new_color_face);
      }
      if (mesh.hasNormalIndices()) {
        const MeshType::Indices::Face& normal_face = mesh.m_FaceIndicesNormals[i];
        MeshType::Indices::Face new_normal_face;
        new_normal_face.setSize(3);
        new_normal_face.setPtr(new unsigned int[3]);
        new_normal_face[0] = old_to_new_normal_indices.at(normal_face[0]);
        new_normal_face[1] = old_to_new_normal_indices.at(normal_face[1]);
        new_normal_face[2] = old_to_new_normal_indices.at(normal_face[2]);
        clipped_mesh.m_FaceIndicesNormals.push_back(new_normal_face);
      }
      if (mesh.hasTexCoordsIndices()) {
        const MeshType::Indices::Face& textcoord_face = mesh.m_FaceIndicesTextureCoords[i];
        MeshType::Indices::Face new_textcoord_face;
        new_textcoord_face.setSize(3);
        new_textcoord_face.setPtr(new unsigned int[3]);
        new_textcoord_face[0] = old_to_new_textcoord_indices.at(textcoord_face[0]);
        new_textcoord_face[1] = old_to_new_textcoord_indices.at(textcoord_face[1]);
        new_textcoord_face[2] = old_to_new_textcoord_indices.at(textcoord_face[2]);
        clipped_mesh.m_FaceIndicesTextureCoords.push_back(new_textcoord_face);
      }
    }
    return clipped_mesh;
  }

  bool run() {
    MeshType mesh;
    MeshIOType::loadFromFile(in_mesh_filename_, mesh);
    cout << "Number of vertices in mesh: " << mesh.m_Vertices.size() << endl;
    cout << "Number of faces in mesh: " << mesh.m_FaceIndicesVertices.size() << endl;
    cout << "Number of colors in mesh: " << mesh.m_Colors.size() << endl;
    cout << "Number of normals in mesh: " << mesh.m_Normals.size() << endl;
    cout << "Number of texture coordinates in mesh: " << mesh.m_TextureCoords.size() << endl;

    MeshType clipped_mesh;
    if (options_.use_region_file) {
      BH_ASSERT(options_.isSet("regions_json_filename"));
      BH_ASSERT(options_.isSet("dense_reconstruction_path"));
      reconstruction::DenseReconstruction dense_reconstruction;
      const bool read_sfm_gps_transformation = true;
      dense_reconstruction.read(options_.dense_reconstruction_path, read_sfm_gps_transformation);
      const GpsCoordinateType gps_reference = dense_reconstruction.sfmGpsTransformation().gps_reference;
      std::cout << "GPS reference: " << gps_reference << std::endl;
      const GpsConverter gps_converter = GpsConverter::createWGS84(gps_reference);

      // Get ROI for clipping point cloud
      boost::property_tree::ptree pt;
      boost::property_tree::read_json(options_.regions_json_filename, pt);
      std::cout << "Region of interest" << std::endl;
      const RegionType roi = convertGpsRegionToEnuRegion(gps_converter, pt.get_child("regionOfInterest"));
      std::vector<RegionType> no_fly_zones;
      std::cout << "ROI bounding box: " << roi.getBoundingBox() << std::endl;
      for (const boost::property_tree::ptree::value_type& v : pt.get_child("noFlyZones")) {
        std::cout << "No-Fly-Zones" << std::endl;
        no_fly_zones.push_back(convertGpsRegionToEnuRegion(gps_converter, v.second));
      }
      clipped_mesh = clipMesh(mesh, [&](const Vector3& point) -> bool {
        const Vector3 point_with_offset = point + options_.offset_vector;
        for (const RegionType& no_fly_zone : no_fly_zones) {
          if (no_fly_zone.isPointInside(point_with_offset)) {
            return false;
          }
        }
        if (roi.isPointOutside(point_with_offset)) {
          return roi.distanceToPoint(point_with_offset) < options_.clip_distance;
        }
        return true;
      });
    }
    else {
      BoundingBoxType clip_bbox(options_.clip_bbox_min, options_.clip_bbox_max);
      clipped_mesh = clipMesh(mesh, [&](const Vector3& point) -> bool {
        const Vector3 point_with_offset = point + options_.offset_vector;
        if (clip_bbox.isOutside(point_with_offset)) {
          return clip_bbox.distanceTo(point_with_offset) < options_.clip_distance;
        }
        return true;
      });
    }

    cout << "Number of vertices in clipped mesh: " << clipped_mesh.m_Vertices.size() << endl;
    cout << "Number of faces in clipped mesh: " << clipped_mesh.m_FaceIndicesVertices.size() << endl;
    cout << "Number of colors in clipped mesh: " << clipped_mesh.m_Colors.size() << endl;
    cout << "Number of normals in clipped mesh: " << clipped_mesh.m_Normals.size() << endl;
    cout << "Number of texture coordinates in clipped mesh: " << clipped_mesh.m_TextureCoords.size() << endl;
    MeshIOType::saveToFile(out_mesh_filename_, clipped_mesh);

    return true;
  }

private:
  Options options_;
  string in_mesh_filename_;
  string out_mesh_filename_;
};

const string ClipAndTransformMeshCmdline::Options::kPrefix = "clip_mesh";

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv, std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("config-file", po::value<string>()->default_value("clip_mesh.cfg"), "Config file.")
        ("in-mesh", po::value<string>(), "File to load the input mesh from.")
        ("out-mesh", po::value<string>()->required(), "File to save the clipped mesh to.")
        ;

    po::options_description options;
    options.add(generic_options);
    po::store(po::command_line_parser(argc, argv).options(options).run(), vm);
    if (vm.count("help")) {
      cout << options << endl;
      return std::make_pair(false, vm);
    }
    po::notify(vm);

    po::options_description config_file_options;
    for (auto& entry : config_options) {
      config_file_options.add(entry.second->getBoostOptions());
    }
    std::ifstream config_in(vm["config-file"].as<string>());
    if (!config_in) {
      throw BH_EXCEPTION("Unable to open config file");
    }
    else {
      po::store(parse_config_file(config_in, config_file_options), vm);
      notify(vm);
    }

    for (auto& entry : config_options) {
      entry.second->setVariablesMap(vm);
    }

    return std::make_pair(true, vm);
  }
  catch (const po::required_option& err) {
    cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << endl;
    return std::make_pair(false, vm);
  }
  catch (const po::error& err) {
    cerr << "Error parsing command line: " << err.what() << endl;
    return std::make_pair(false, vm);
  }
}

bool ctrl_c_pressed = false;
void signalIntHandler(int sig) {
  ctrl_c_pressed = true;
}

void enableCtrlCHandler(void (*signalHandler)(int)) {
  std::signal(SIGINT, signalHandler);
}

void disableCtrlCHandler() {
  std::signal(SIGINT, SIG_DFL);
}

int main(int argc, char** argv)
{
  std::map<std::string, std::unique_ptr<bh::ConfigOptions>> config_options =
      ClipAndTransformMeshCmdline::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
      processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
      return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  ClipAndTransformMeshCmdline clip_cmdline(
      config_options, vm["in-mesh"].as<string>(), vm["out-mesh"].as<string>());

  if (clip_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
