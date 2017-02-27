//==================================================
// transform_mesh.cpp
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
#include <ait/eigen_options.h>
#include <ait/geometry.h>

#include <ait/mLib.h>
#include <ait/mLibUtils.h>

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
//
//namespace boost {
//void validate(boost::any& v,
//              const std::vector<std::string>& values,
//              Vector3* target_type, int) {
////    static boost::regex r("\\d\\d\\d-(\\d\\d\\d)");
//
//    using namespace boost::program_options;
//
//    // Make sure no previous assignment to 'v' was made.
//    validators::check_first_occurrence(v);
//    if (values.size() != 3) {
//      boost::throw_exception(validation_error(validation_error::invalid_option_value));
//    }
//
//    Vector3 vector;
//    for (size_t i = 0; i < static_cast<size_t>(vector.size()); ++i) {
//      vector(i) = boost::lexical_cast<FloatType>(values[i]);
//    }
//    v = boost::any(vector);
//    cout << vector << endl;
//}
//}

class TransformMeshCmdline {
public:

  class Options : public ait::ConfigOptions {
  public:
    static const string kPrefix;

    Options()
    : ait::ConfigOptions(kPrefix) {
      addOption<Vector3>("scale_before_transformation", &scale_before_transformation);
      addOption<Vector3>("translation_before_rotation", &translation_before_rotation);
      addOption<Vector3>("translation", &translation);
      addOption<Vector3>("rotation_xaxis", &rotation_xaxis);
      addOption<Vector3>("rotation_yaxis", &rotation_yaxis);
      addOption<Vector3>("rotation_zaxis", &rotation_zaxis);
      addOption<Vector3>("scale", &scale);
      addOption<bool>("keep_colors", &keep_colors);
      addOption<bool>("keep_normals", &keep_normals);
      addOption<bool>("keep_texcoords", &keep_texcoords);
    }

    ~Options() override {}

    Vector3 scale_before_transformation = Vector3::Ones();
    Vector3 translation_before_rotation = Vector3::Zero();
    Vector3 translation = Vector3::Zero();
    Vector3 rotation_xaxis = Vector3::UnitX();
    Vector3 rotation_yaxis = Vector3::UnitY();
    Vector3 rotation_zaxis = Vector3::UnitZ();
    Vector3 scale = Vector3::Ones();
    bool keep_colors = true;
    bool keep_normals = true;
    bool keep_texcoords = true;
  };

  static std::map<string, std::unique_ptr<ait::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<ait::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
        std::forward_as_tuple(Options::kPrefix),
        std::forward_as_tuple(static_cast<ait::ConfigOptions*>(new Options())));
    return config_options;
  }

  TransformMeshCmdline(const std::map<string, std::unique_ptr<ait::ConfigOptions>>& config_options,
      const string& in_mesh_filename, const string& out_mesh_filename)
  : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())),
    in_mesh_filename_(in_mesh_filename),
    out_mesh_filename_(out_mesh_filename) {}

  ~TransformMeshCmdline() {
  }

  bool run() {
    Matrix3x3 rot_eigen;
    AIT_PRINT_VALUE(options_.scale_before_transformation);
    AIT_PRINT_VALUE(options_.translation_before_rotation);
    rot_eigen.col(0) = options_.rotation_xaxis;
    rot_eigen.col(1) = options_.rotation_yaxis;
    rot_eigen.col(2) = options_.rotation_zaxis;
    AIT_PRINT_VALUE(options_.translation);
    AIT_PRINT_VALUE(rot_eigen);
    AIT_PRINT_VALUE(options_.scale);
    const ml::Matrix3x3<FloatType> rot = ait::MLibUtilities::convertEigenToMlib(rot_eigen);
    const ml::vec3<FloatType> translation = ait::MLibUtilities::convertEigenToMlib(options_.translation);
    const ml::vec3<FloatType> translation_before_rotation =
        ait::MLibUtilities::convertEigenToMlib(options_.translation_before_rotation);

    MeshType mesh;
    MeshIOType::loadFromFile(in_mesh_filename_, mesh);
    cout << "Number of vertices in mesh: " << mesh.m_Vertices.size() << endl;
    cout << "Number of faces in mesh: " << mesh.m_FaceIndicesVertices.size() << endl;
    cout << "Number of colors in mesh: " << mesh.m_Colors.size() << endl;
    cout << "Number of normals in mesh: " << mesh.m_Normals.size() << endl;
    cout << "Number of texture coordinates in mesh: " << mesh.m_TextureCoords.size() << endl;

    MeshType transformed_mesh;
    for (std::size_t i = 0; i < mesh.m_Vertices.size(); ++i) {
      const ml::vec3<FloatType>& vertex = mesh.m_Vertices[i];
      ml::vec3<FloatType> transformed_vertex;
      transformed_vertex = vertex;
      transformed_vertex.x *= options_.scale_before_transformation(0);
      transformed_vertex.y *= options_.scale_before_transformation(1);
      transformed_vertex.z *= options_.scale_before_transformation(2);
      transformed_vertex += translation_before_rotation;
      transformed_vertex = rot * transformed_vertex;
      transformed_vertex += translation;
      transformed_vertex.x *= options_.scale(0);
      transformed_vertex.y *= options_.scale(1);
      transformed_vertex.z *= options_.scale(2);
      transformed_mesh.m_Vertices.push_back(transformed_vertex);
      if (options_.keep_colors && mesh.hasColors()) {
        transformed_mesh.m_Colors.push_back(mesh.m_Colors[i]);
      }
      if (options_.keep_normals && mesh.hasNormals()) {
        const ml::vec3<FloatType>& normal = mesh.m_Normals[i];
        ml::vec3<FloatType> transformed_normal;
        transformed_normal = rot * normal;
        transformed_mesh.m_Normals.push_back(transformed_normal);
      }
      if (options_.keep_texcoords && mesh.hasTexCoords()) {
        transformed_mesh.m_TextureCoords.push_back(mesh.m_TextureCoords[i]);
      }
    }
    for (std::size_t i = 0; i < mesh.m_FaceIndicesVertices.size(); ++i) {
      const MeshType::Indices::Face& face = mesh.m_FaceIndicesVertices[i];
      AIT_ASSERT_STR(face.size() == 3, "Mesh faces need to have a valence of 3");
      transformed_mesh.m_FaceIndicesVertices.push_back(face);
      if (options_.keep_colors && mesh.hasColors()) {
        transformed_mesh.m_FaceIndicesVertices.push_back(mesh.m_FaceIndicesColors[i]);
      }
      if (options_.keep_normals && mesh.hasNormals()) {
        transformed_mesh.m_FaceIndicesNormals.push_back(mesh.m_FaceIndicesNormals[i]);
      }
      if (options_.keep_texcoords && mesh.hasTexCoords()) {
        transformed_mesh.m_FaceIndicesTextureCoords.push_back(mesh.m_FaceIndicesTextureCoords[i]);
      }
    }
    cout << "Number of vertices in transformed mesh: " << transformed_mesh.m_Vertices.size() << endl;
    cout << "Number of faces in transformed mesh: " << transformed_mesh.m_FaceIndicesVertices.size() << endl;
    cout << "Number of colors in transformed mesh: " << transformed_mesh.m_Colors.size() << endl;
    cout << "Number of normals in transformed mesh: " << transformed_mesh.m_Normals.size() << endl;
    cout << "Number of texture coordinates in transformed mesh: " << transformed_mesh.m_TextureCoords.size() << endl;
    MeshIOType::saveToFile(out_mesh_filename_, transformed_mesh);

    return true;
  }

private:
  Options options_;
  string in_mesh_filename_;
  string out_mesh_filename_;
};

const string TransformMeshCmdline::Options::kPrefix = "transform_mesh";

std::pair<bool, boost::program_options::variables_map> processOptions(
    int argc, char** argv, std::map<string, std::unique_ptr<ait::ConfigOptions>>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
        ("help", "Produce help message")
        ("config-file", po::value<string>()->default_value("transform_mesh.cfg"), "Config file.")
        ("in-mesh", po::value<string>(), "File to load the input mesh from.")
        ("out-mesh", po::value<string>()->required(), "File to save the transformed mesh to.")
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
      TransformMeshCmdline::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
      processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
      return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  TransformMeshCmdline transform_cmdline(
      config_options, vm["in-mesh"].as<string>(), vm["out-mesh"].as<string>());

  if (transform_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
