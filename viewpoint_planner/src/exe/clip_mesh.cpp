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

#include <ait/boost.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <ait/common.h>
#include <ait/eigen.h>
#include <ait/utilities.h>
#include <ait/options.h>
#include <ait/math/geometry.h>

#include <ait/mLib.h>

using std::cout;
using std::cerr;
using std::endl;
using std::string;

using FloatType = float;
USE_FIXED_EIGEN_TYPES(FloatType)

using BoundingBoxType = ait::BoundingBox3D<FloatType>;
using MeshType = ml::MeshData<FloatType>;
using MeshIOType = ml::MeshIO<FloatType>;
using TriMeshType = ml::TriMesh<FloatType>;

class ClipAndTransformMeshCmdline {
public:

  class Options : public ait::ConfigOptions {
  public:
    static const string kPrefix;

    Options()
    : ait::ConfigOptions(kPrefix) {
      addOption<FloatType>("clip_bbox_min_x");
      addOption<FloatType>("clip_bbox_min_y");
      addOption<FloatType>("clip_bbox_min_z");
      addOption<FloatType>("clip_bbox_max_x");
      addOption<FloatType>("clip_bbox_max_y");
      addOption<FloatType>("clip_bbox_max_z");
    }

    ~Options() override {}
  };

  static std::map<string, std::unique_ptr<ait::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<ait::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<ait::ConfigOptions*>(new Options())));
    return config_options;
  }

  ClipAndTransformMeshCmdline(const std::map<string, std::unique_ptr<ait::ConfigOptions>>& config_options,
      const string& in_mesh_filename, const string& out_mesh_filename)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    in_mesh_filename_(in_mesh_filename),
    out_mesh_filename_(out_mesh_filename) {}

  ~ClipAndTransformMeshCmdline() {
  }

  bool run() {
    BoundingBoxType clip_bbox(
        Vector3(options_.getValue<FloatType>("clip_bbox_min_x"),
            options_.getValue<FloatType>("clip_bbox_min_y"),
            options_.getValue<FloatType>("clip_bbox_min_z")),
        Vector3(options_.getValue<FloatType>("clip_bbox_max_x"),
            options_.getValue<FloatType>("clip_bbox_max_y"),
            options_.getValue<FloatType>("clip_bbox_max_z")));

    MeshType mesh;
    MeshIOType::loadFromFile(in_mesh_filename_, mesh);
    cout << "Number of vertices in mesh: " << mesh.m_Vertices.size() << endl;
    cout << "Number of faces in mesh: " << mesh.m_FaceIndicesVertices.size() << endl;
    cout << "Number of colors in mesh: " << mesh.m_Colors.size() << endl;
    cout << "Number of normals in mesh: " << mesh.m_Normals.size() << endl;
    cout << "Number of texture coordinates in mesh: " << mesh.m_TextureCoords.size() << endl;
    MeshType clipped_mesh;
    std::unordered_map<size_t, size_t> old_to_new_vertex_indices;
    std::unordered_map<size_t, size_t> old_to_new_color_indices;
    std::unordered_map<size_t, size_t> old_to_new_normal_indices;
    std::unordered_map<size_t, size_t> old_to_new_textcoord_indices;
    for (std::size_t i = 0; i < mesh.m_Vertices.size(); ++i) {
      const ml::vec3<FloatType>& ml_vertex = mesh.m_Vertices[i];
      const Vector3 vertex(ml_vertex.x, ml_vertex.y, ml_vertex.z);
      if (clip_bbox.isInside(vertex)) {
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
      AIT_ASSERT_STR(vertex_face.size() == 3, "Mesh faces need to have a valence of 3");
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
    int argc, char** argv, std::map<string, std::unique_ptr<ait::ConfigOptions>>& config_options)
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
      throw AIT_EXCEPTION("Unable to open config file");
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
  std::map<std::string, std::unique_ptr<ait::ConfigOptions>> config_options =
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
