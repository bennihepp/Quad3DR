//==================================================
// image_matcher.cpp.cpp
//
//  Copyright (c) 2017 Benjamin Hepp.
//  Author: Benjamin Hepp
//  Created on: 03.04.17
//==================================================

#include <iostream>
#include <memory>
#include <csignal>

#include <bh/boost.h>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>
#include <boost/functional/hash.hpp>

#include <bh/common.h>
#include <bh/eigen.h>
#include <bh/utilities.h>
#include <bh/math/utilities.h>
#include <bh/config_options.h>
#include <bh/filesystem.h>
#include <bh/eigen_options.h>
#include <bh/sqlite/sqlite3_wrapper.h>
#include <bh/math/geometry.h>
#include <bh/mLib/mLibUtils.h>
#include <bh/nn/approximate_nearest_neighbor.h>

#include <bh/mLib/mLib.h>

#if WITH_OPENGL_OFFSCREEN
#include <QApplication>
#endif

#include <bh/gps.h>
#include <bh/string_utils.h>
#include <bh/pose.h>
#include <bh/se3_transform.h>
#include <bh/vision/types.h>
#include <bh/vision/geometry.h>
#include <bh/vision/cameras.h>
#include <bh/vision/drawing_qt.h>
#include <bh/opencv/matrix.h>
#include <bh/opencv/images.h>
#include <bh/opencv/drawing.h>

#include "../planner/viewpoint_offscreen_renderer.h"

using std::cout;
using std::cerr;
using std::endl;
using std::string;

namespace boostfs = boost::filesystem;

class ImageMatcherCmdline {
public:
  using FloatType = float;
  USE_FIXED_EIGEN_TYPES(FloatType);

  using BoundingBoxType = bh::BoundingBox3D<FloatType>;
  using PointCloudType = ml::PointCloud<FloatType>;
  using PointCloudIOType = ml::PointCloudIO<FloatType>;
  using MeshDataType = ml::MeshData<FloatType>;
  using TriMeshType = ml::TriMesh<FloatType>;
  using MeshIOType = ml::MeshIO<FloatType>;
  using RayType = ml::Ray<FloatType>;
  using TriMeshAcceleratorType = ml::TriMeshAcceleratorBVH<FloatType>;
  using Triangle = bh::Triangle<FloatType>;
  using ViewpointOffscreenRenderer = viewpoint_planner::ViewpointOffscreenRenderer;

  struct Options : bh::ConfigOptions {
    static const string kPrefix;

    Options()
            : bh::ConfigOptions(kPrefix) {
      addOptionRequired<string>("dense_reconstruction_path", &dense_reconstruction_path);
      addOptionRequired<string>("poisson_mesh_filename", &poisson_mesh_filename);
      addOptionRequired<string>("images_path", &images_path);
      addOptionRequired<string>("image_prior_filename", &image_prior_filename);
      addOptionRequired<string>("colmap_db_filename", &colmap_db_filename);
      addOption<FloatType>("virtual_camera_width", &virtual_camera_width);
      addOption<FloatType>("virtual_camera_height", &virtual_camera_height);
      addOption<FloatType>("virtual_camera_focal_length", &virtual_camera_focal_length);
//      addOption<FloatType>("depth_tolerance", &depth_tolerance);
      addOption<FloatType>("min_depth", &min_depth);
      addOption<FloatType>("max_depth", &max_depth);
      addOption<FloatType>("sampson_distance_threshold", &sampson_distance_threshold);
      addOption<FloatType>("max_projection_error", &max_projection_error);
      addOption<FloatType>("max_feature_distance", &max_feature_distance);
      addOption<FloatType>("lowe_ratio_threshold", &lowe_ratio_threshold);
      addOption<FloatType>("max_angular_distance_to_prior_degrees", &max_angular_distance_to_prior_degrees);
      addOption<FloatType>("min_translation_dot_product_with_prior", &min_translation_dot_product_with_prior);
      addOption<bool>("cross_check", &cross_check);
      addOption<FloatType>("max_epipolar_error", &max_epipolar_error);
      addOption<size_t>("min_num_inliers", &min_num_inliers);
      addOption<FloatType>("cmap_min_depth", &cmap_min_depth);
      addOption<FloatType>("cmap_max_depth", &cmap_max_depth);
      addOption<size_t>("num_ransac_iterations", &num_ransac_iterations);
      addOption<bool>("guided_matching", &guided_matching);
      addOption<bool>("delete_feature_matches_in_db", &delete_feature_matches_in_db);
      addOption<bool>("dump_prior_mesh_images", &dump_prior_mesh_images);
      addOption<bool>("dump_keypoint_images", &dump_keypoint_images);
      addOption<bool>("dump_match_images", &dump_match_images);
      addOption<bool>("dump_epipolar_lines", &dump_epipolar_lines);
    }

    ~Options() override {}

    string dense_reconstruction_path;
    string poisson_mesh_filename;
    string images_path;
    string image_prior_filename;
    string colmap_db_filename;
    FloatType virtual_camera_width = 600;
    FloatType virtual_camera_height = 450;
    FloatType virtual_camera_focal_length = FloatType(345.2895);
//    FloatType depth_tolerance = FloatType(0.5);
    FloatType min_depth = FloatType(0.0);
    FloatType max_depth = FloatType(50.0);
    FloatType sampson_distance_threshold = FloatType(0.05);
    FloatType max_projection_error = FloatType(5.0);
    FloatType max_feature_distance = FloatType(0.7);
    FloatType lowe_ratio_threshold = FloatType(0.2);
    FloatType max_angular_distance_to_prior_degrees = FloatType(30.0);
    FloatType min_translation_dot_product_with_prior = FloatType(3.0);
    bool cross_check = true;
    FloatType max_epipolar_error = FloatType(4.0);
    size_t min_num_inliers = 15;
    FloatType cmap_min_depth = FloatType(0);
    FloatType cmap_max_depth = FloatType(100);
    size_t num_ransac_iterations = 10000;
    bool guided_matching = true;
    bool delete_feature_matches_in_db = false;
    bool dump_prior_mesh_images = false;
    bool dump_keypoint_images = false;
    bool dump_match_images = false;
    bool dump_epipolar_lines = false;
  };

  using ImageId = int;
  using CameraId = int;

  using SE3Transform = bh::SE3Transform<FloatType>;
  using PoseType = bh::Pose<FloatType>;

  using Keypoint = bh::vision::Keypoint<FloatType>;
  using KeypointMatch = bh::vision::KeypointMatch<FloatType>;

  BH_USE_VISION_GEOMETRY_TYPES(FloatType);
  using OpenCVCameraType = bh::vision::OpenCVCamera<FloatType>;
  using CameraIdToOpenCVCameraMap = EIGEN_ALIGNED_UNORDERED_MAP(CameraId, OpenCVCameraType);
  using ImageIdToPoseMap = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, PoseType);

  struct SiftDescriptor {
    Eigen::Matrix<uint8_t, 128, 1> desc;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  };

  using SiftScore = int;
  using SiftMatchMatrix = bh::EigenTypes<SiftScore>::MatrixDynamic;

  using SiftDescriptorVector = EIGEN_ALIGNED_VECTOR(SiftDescriptor);
  using ImageIdToSiftDescriptorMap = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, SiftDescriptor);
  using ImageIdToSiftDescriptorVectorMap = EIGEN_ALIGNED_UNORDERED_MAP(ImageId, SiftDescriptorVector);

  static std::map<string, std::unique_ptr<bh::ConfigOptions>> getConfigOptions() {
    std::map<string, std::unique_ptr<bh::ConfigOptions>> config_options;
    config_options.emplace(std::piecewise_construct,
                           std::forward_as_tuple(Options::kPrefix),
                           std::forward_as_tuple(static_cast<bh::ConfigOptions*>(new Options())));
    return config_options;
  }

  ImageMatcherCmdline(
          const std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options)
          : options_(*dynamic_cast<Options*>(config_options.at(Options::kPrefix).get())) {
    sqlite_db_.reset(new bh::SQLite3(options_.colmap_db_filename, bh::SQLite3::READWRITE));
  }

  ~ImageMatcherCmdline() {
  }

  std::vector<string> getImageFilenames() const {
    std::vector<string> image_filenames;
    boostfs::path images_path(options_.images_path);
    try {
      if (!boostfs::exists(images_path)) {
        throw bh::Error("Path to images does not exist");
      }
      if (!boostfs::is_directory(images_path)) {
        throw bh::Error("Path to images is not a directory");
      }
      for (auto it = boostfs::directory_iterator(images_path); it != boostfs::directory_iterator(); ++it) {
        if (boostfs::is_regular_file(it->status())) {
          image_filenames.push_back(bh::pathRelativeTo(images_path, it->path()).string());
        }
      }
    }
    catch (const boostfs::filesystem_error& ex) {
      throw bh::Error(ex.what());
    }
    return image_filenames;
  }

  std::vector<ImageId> getImageIdsFromColmap() {
    std::vector<ImageId> image_ids;
    const string query = "SELECT image_id FROM images";
    sqlite_db_->execute(
            query,
            [&](const bh::SQLite3::RowResult& row_result) {
              image_ids.push_back(row_result.getInt(0));
            });
    return image_ids;
  }

//  string getImageNameFromColmap(const ImageId image_id) {
//    const string query = string("SELECT name FROM images WHERE image_id == ") + std::to_string(image_id);
//    string image_name;
//    sqlite_db_->execute(
//            query,
//            [&](const bh::SQLite3::RowResult& row_result) {
//              image_name = row_result.getString(0);
//            });
//    return image_name;
//  }

  std::unordered_map<ImageId, string> getImageNamesFromColmap() {
    std::unordered_map<ImageId, string> image_names;
    const string query = "SELECT image_id, name FROM images";
    sqlite_db_->execute(
            query,
            [&](const bh::SQLite3::RowResult& row_result) {
              const ImageId image_id = row_result.getInt(0);
              const string name = row_result.getString(1);
              image_names.emplace(image_id, name);
            });
    return image_names;
  };

  std::unordered_map<ImageId, CameraId> getImageCameraIdsFromColmap() {
    std::unordered_map<ImageId, CameraId> image_camera_ids;
    const string query = "SELECT image_id, camera_id FROM images";
    sqlite_db_->execute(
            query,
            [&](const bh::SQLite3::RowResult& row_result) {
              const ImageId image_id = row_result.getInt(0);
              const CameraId camera_id = row_result.getInt(1);
              image_camera_ids.emplace(image_id, camera_id);
            });
    return image_camera_ids;
  };

  CameraIdToOpenCVCameraMap getCamerasFromColmap() {
    CameraIdToOpenCVCameraMap cameras_;
    const string query = "SELECT camera_id, model, width, height, params FROM cameras";
    sqlite_db_->execute(
            query,
            [&](const bh::SQLite3::RowResult& row_result) {
              const CameraId camera_id = row_result.getInt(0);
              const int model = row_result.getInt(1);
              OpenCVCameraType opencv_camera;
              opencv_camera.width() = (size_t)row_result.getInt(2);
              opencv_camera.height() = (size_t)row_result.getInt(3);
              std::vector<double> params = row_result.getData<double>(4);
              if (model == 2) {
                BH_ASSERT(params.size() == 4);
                const double focal_length = params[0];
                opencv_camera.intrinsics()(0, 0) = focal_length;
                opencv_camera.intrinsics()(1, 1) = focal_length;
                const double cx = params[1];
                opencv_camera.intrinsics()(0, 2) = cx;
                const double cy = params[2];
                opencv_camera.intrinsics()(1, 2) = cy;
                const double k = params[3];
                opencv_camera.k1() = k;
//                cout << "Camera intrinsics: " << opencv_camera.intrinsics() << endl;
              }
              else {
                throw bh::Error(std::string("Camera model not supported: ") + std::to_string(model));
              }
              cameras_.emplace(camera_id, opencv_camera);
            });
    return cameras_;
  };

  std::unordered_map<ImageId, std::vector<Keypoint>> getKeypointsFromColmap() {
    std::unordered_map<ImageId, std::vector<Keypoint>> all_keypoints;
    const string query = "SELECT image_id, rows, cols, data FROM keypoints";
    sqlite_db_->execute(
            query,
            [&](const bh::SQLite3::RowResult& row_result) {
              const ImageId image_id = row_result.getInt(0);
              const int rows = row_result.getInt(1);
              const int cols = row_result.getInt(2);
              BH_ASSERT(cols == 4);
              std::vector<Keypoint> keypoints;
              keypoints.reserve(rows);
              std::vector<float> data = row_result.getData<float>(3);
              BH_ASSERT(data.size() == size_t(rows * cols));
              for (size_t row = 0; row < (size_t)rows; ++row) {
                const FloatType x = data[row * cols + 0];
                const FloatType y = data[row * cols + 1];
                const FloatType scale = data[row * cols + 2];
                const FloatType orientation = data[row * cols + 3];
                Keypoint kp(x, y, scale, orientation);
                keypoints.push_back(kp);
              }
              all_keypoints.emplace(image_id, keypoints);
            });
    return all_keypoints;
  };

  ImageIdToSiftDescriptorVectorMap getDescriptorsFromColmap() {
    ImageIdToSiftDescriptorVectorMap all_descriptors;
    const string query = "SELECT image_id, rows, cols, data FROM descriptors";
    sqlite_db_->execute(
            query,
            [&](const bh::SQLite3::RowResult& row_result) {
              const ImageId image_id = row_result.getInt(0);
              const int rows = row_result.getInt(1);
              const int cols = row_result.getInt(2);
              BH_ASSERT(cols == 128);
              SiftDescriptorVector descriptors;
              descriptors.reserve(rows);
              std::vector<uint8_t> data = row_result.getData<uint8_t>(3);
              BH_ASSERT(data.size() == size_t(rows * cols));
              for (size_t row = 0; row < (size_t)rows; ++row) {
                SiftDescriptor sd;
                std::memcpy(sd.desc.data(), &data[0] + row * cols, sd.desc.size() * sizeof(sd.desc[0]));
                descriptors.push_back(sd);
              }
              all_descriptors.emplace(image_id, descriptors);
            });
    return all_descriptors;
  };

//  std::vector<Keypoint> undistortKeypoints(const OpenCVCameraType& camera, const std::vector<Keypoint>& keypoints) {
//    std::vector<Keypoint> undist_keypoints;
//    undist_keypoints.reserve(keypoints.size());
//    for (const Keypoint& keypoint : keypoints) {
//      const Vector2 point(keypoint.x, keypoint.y);
//      const Vector2 dist_world_point = camera.imageToWorldWithoutUndistortion(point);
//      const Vector2 undist_world_point = camera.undistortWorldPoint(dist_world_point);
//      const Vector2 undist_image_point = camera.worldToImageWithoutDistortion(undist_world_point);
//      Keypoint undist_keypoint = keypoint;
//      undist_keypoint.x = undist_image_point(0);
//      undist_keypoint.y = undist_image_point(1);
//      undist_keypoints.push_back(undist_keypoint);
//      cout << "undistorted from " << point.transpose() << " -> " << undist_image_point.transpose() << endl;
//      const Vector2 redist_world_point = camera.distortWorldPoint(undist_world_point);
//      cout << "dist_point " << dist_world_point.transpose() << endl;
//      cout << "undist_point " << undist_world_point.transpose() << endl;
//      cout << "redist_world_point " << redist_world_point.transpose() << endl;
//      cout << (redist_world_point - dist_world_point).squaredNorm() << endl;
//    }
//    return undist_keypoints;
//  };

  ImageIdToPoseMap readPriorImagePoses() const {
    std::vector<string> image_filenames = getImageFilenames();
    std::sort(std::begin(image_filenames), std::end(image_filenames),
              [&](const string& a, const string& b) {
                return std::lexicographical_compare(std::begin(a), std::end(a), std::begin(b), std::end(b));
              });
//    // Debugging: Print image filenames
//    cout << "Image files: " << endl;
//    std::for_each(image_filenames.begin(), image_filenames.end(), [&](const string& filename) {
//      cout << "  " << filename << endl;
//    });

    std::unordered_map<size_t, ImageId> file_to_image_id_map;
    using ImageIdToNameMapValue = typename std::unordered_map<ImageId, string>::value_type;
    for (auto it = std::begin(image_filenames); it != std::end(image_filenames); ++it) {
      const auto image_id_to_name_map_it = std::find_if(
              std::begin(image_names_), std::end(image_names_),
              [&](const ImageIdToNameMapValue& value) {
                return value.second == *it;
              });
      if (image_id_to_name_map_it != std::end(image_names_)) {
        const ImageId image_id = image_id_to_name_map_it->first;
        file_to_image_id_map.emplace(it - std::begin(image_filenames), image_id);
      }
      else {
        throw bh::Error(string("Could not find image id to match filename ") + *it);
      }
    }

    using GpsFloatType = double;
    using GpsConverterType = bh::GpsConverter<GpsFloatType>;
    reconstruction::DenseReconstruction dense_reconstruction;
    const bool read_sfm_gps_transformation = true;
    dense_reconstruction.read(options_.dense_reconstruction_path, read_sfm_gps_transformation);
    GpsConverterType gps_converter = GpsConverterType::createWGS84(
            dense_reconstruction.sfmGpsTransformation().gps_reference.cast<GpsFloatType>());

    ImageIdToPoseMap poses;
    std::ifstream ifs(options_.image_prior_filename);
    if (!ifs) {
      throw bh::Error(string("Unable to open image prior file: " ) + options_.image_prior_filename);
    }
    while (ifs) {
      std::string line;
      std::string item;

      while (std::getline(ifs, line)) {
        bh::trim(line);

        if (line.empty() || line[0] == '#') {
          continue;
        }

        std::stringstream line_stream(line);

        // Prior image ID
        std::getline(line_stream, item, ' ');
        bh::trim(item);
        const size_t prior_image_id = boost::lexical_cast<size_t>(item);
        BH_ASSERT(file_to_image_id_map.count(prior_image_id) > 0);
        const ImageId image_id = file_to_image_id_map.at(prior_image_id);
//        cout << "image_id=" << image_id << ", prior_image_id=" << prior_image_id << endl;
        // Latitude
        std::getline(line_stream, item, ' ');
        bh::trim(item);
        const FloatType latitude = boost::lexical_cast<FloatType>(item);
        // Longitude
        std::getline(line_stream, item, ' ');
        bh::trim(item);
        const FloatType longitude = boost::lexical_cast<FloatType>(item);
        // Height
        std::getline(line_stream, item, ' ');
        bh::trim(item);
        const FloatType height = boost::lexical_cast<FloatType>(item);
        // Yaw
        std::getline(line_stream, item, ' ');
        bh::trim(item);
//        const FloatType yaw_degrees = boost::lexical_cast<FloatType>(item);
//        const FloatType yaw = bh::degreeToRadians(yaw_degrees);
        // Gimbal yaw (in ground coordinate system, NED)
        std::getline(line_stream, item, ' ');
        bh::trim(item);
        const FloatType gimbal_yaw_degrees = boost::lexical_cast<FloatType>(item);
        const FloatType gimbal_yaw = bh::degreeToRadians(gimbal_yaw_degrees);
        // Gimbal pitch
        std::getline(line_stream, item, ' ');
        bh::trim(item);
        const FloatType gimbal_pitch_degrees = boost::lexical_cast<FloatType>(item);
        const FloatType gimbal_pitch = bh::degreeToRadians(gimbal_pitch_degrees);
        // Gimbal roll
        std::getline(line_stream, item, ' ');
        bh::trim(item);
        const FloatType gimbal_roll_degrees = boost::lexical_cast<FloatType>(item);
        const FloatType gimbal_roll = bh::degreeToRadians(gimbal_roll_degrees);

        GpsConverterType::GpsCoordinateType gps_coordinate(latitude, longitude, height);
        const Vector3 enu_coordinate = gps_converter.convertGpsToEnu(gps_coordinate).cast<FloatType>();
        const Vector3 translation = enu_coordinate;
        const FloatType gimbal_yaw_enu = gimbal_yaw - M_PI / 2;
        const Quaternion quaternion = AngleAxis(gimbal_yaw_enu, Vector3::UnitY())
                                      * AngleAxis(gimbal_pitch, Vector3::UnitX())
                                      * AngleAxis(gimbal_roll, Vector3::UnitZ());
        // Rotate xyz-pose to camera with z in viewing direction and y pointing downwards.
        const Quaternion euler_to_camera_quat = AngleAxis(- M_PI / 2, Vector3::UnitZ())
                                                * AngleAxis(- M_PI / 2, Vector3::UnitX());
        const PoseType pose(translation, euler_to_camera_quat * quaternion);
        poses.emplace(image_id, pose);
      }
    }
    return poses;
  }

#pragma GCC optimize("O3")
  SiftScore computeMatchScore(const SiftDescriptor& descriptor1, const SiftDescriptor& descriptor2) const {
    const Eigen::Matrix<SiftScore, 128, 1> int_descriptor1 = descriptor1.desc.cast<SiftScore>();
    const Eigen::Matrix<SiftScore, 128, 1> int_descriptor2 = descriptor2.desc.cast<SiftScore>();
    const SiftScore score = int_descriptor1.dot(int_descriptor2);
//    const FloatType score = (descriptor1.desc - descriptor2.desc).squaredNorm();
//    FloatType score = 0;
//    for (size_t i = 0; i < descriptor1.desc.size(); ++i) {
//      const FloatType diff = descriptor1.desc[i] - descriptor2.desc[i];
//      score += diff * diff;
//    }
    return score;
  }

#pragma GCC optimize("O3")
  SiftMatchMatrix computeMatchScores(
          const SiftDescriptorVector& descriptors1, const SiftDescriptorVector& descriptors2) const {
    SiftMatchMatrix match_scores(descriptors1.size(), descriptors2.size());
#pragma omp parallel for
    for (size_t row = 0; row < match_scores.rows(); ++row) {
//      cout << "Descriptor " << i << endl;
      for (size_t col = 0; col < match_scores.cols(); ++col) {
        match_scores(row, col) = computeMatchScore(descriptors1[row], descriptors2[col]);
      }
    }
    return match_scores;
  }

#pragma GCC optimize("O3")
  template <typename GuidedFilter>
  SiftMatchMatrix computeMatchScoresGuided(
          const SiftDescriptorVector& descriptors1,
          const SiftDescriptorVector& descriptors2,
          GuidedFilter&& guided_filter) const {
    SiftMatchMatrix match_scores(descriptors1.size(), descriptors2.size());
#pragma omp parallel for
    for (size_t row = 0; row < match_scores.rows(); ++row) {
//      cout << "Descriptor " << i << endl;
      for (size_t col = 0; col < match_scores.cols(); ++col) {
        if (std::forward<GuidedFilter>(guided_filter)(row, col)) {
          match_scores(row, col) = computeMatchScore(descriptors1[row], descriptors2[col]);
        }
        else {
          match_scores(row, col) = 0;
        }
      }
    }
    return match_scores;
  }

  void invalidateMatchesBasedOnPriorGeometry(const ImageId image_id1, const ImageId image_id2,
                                             SiftMatchMatrix& match_scores,
                                             const FloatType min_depth,
                                             const FloatType max_depth,
                                             const FloatType max_projection_error) {
//    const FloatType max_reprojection_error_squared = max_reprojection_error * max_reprojection_error;
    const FloatType max_projection_error_squared = max_projection_error * max_projection_error;
    const PoseType pose1 = prior_image_poses_.at(image_id1);
    const OpenCVCameraType cv_camera1 = cameras_.at(image_camera_ids_.at(image_id1));
    const OpenCVCameraType virtual_camera1 = OpenCVCameraType(
            cv_camera1.width(), cv_camera1.height(), cv_camera1.intrinsics());
    const PinholeCamera camera1(
            cv_camera1.width(), cv_camera1.height(), cv_camera1.intrinsics());
    offscreen_renderer_->setCamera(camera1);
    QImage depth_image1 = offscreen_renderer_->drawPoissonMeshDepth(pose1);
    const PoseType pose2 = prior_image_poses_.at(image_id2);
    const OpenCVCameraType cv_camera2 = cameras_.at(image_camera_ids_.at(image_id2));
    const OpenCVCameraType virtual_camera2 = OpenCVCameraType(
            cv_camera2.width(), cv_camera2.height(), cv_camera2.intrinsics());
    const PinholeCamera camera2(
            cv_camera2.width(), cv_camera2.height(), cv_camera2.intrinsics());
    offscreen_renderer_->setCamera(camera2);
    QImage depth_image2 = offscreen_renderer_->drawPoissonMeshDepth(pose2);
    const std::vector<Keypoint>& keypoints1 = all_undist_keypoints_.at(image_id1);
    const std::vector<Keypoint>& keypoints2 = all_undist_keypoints_.at(image_id2);
#pragma omp parallel for
    for (size_t row = 0; row < (size_t)match_scores.rows(); ++row) {
      const FloatType x1 = keypoints1[row].x();
      const FloatType y1 = keypoints1[row].y();
      const Vector2 world_point1 = cv_camera1.imageToWorld(Vector2(x1, y1));
      const Vector2 unprojected_point1 = virtual_camera1.worldToImage(world_point1);
      if (!cv_camera1.isInsideImage(unprojected_point1)) {
        match_scores.row(row).setConstant(0);
        continue;
      }
      const FloatType depth1 = offscreen_renderer_->decodeDepthValue(depth_image1, unprojected_point1);
      if (depth1 > max_depth || depth1 <= min_depth) {
        match_scores.row(row).setConstant(0);
        continue;
      }
      const Vector3 world_point_3d_1 = depth1 * world_point1.homogeneous();
      const Vector3 abs_world_point_3d_1 = pose1.getTransformationImageToWorld() * world_point_3d_1.homogeneous();
//      const Vector3 world_point_3d_2 = pose2.getTransformationWorldToImage() * abs_world_point_3d_1.homogeneous();
//      const Vector2 world_point2(world_point_3d_2.hnormalized());
//      const Vector2 image_point2 = cv_camera2.worldToImage(world_point2);
      for (size_t col = 0; col < (size_t)match_scores.cols(); ++col) {
        const FloatType x2 = keypoints2[col].x();
        const FloatType y2 = keypoints2[col].y();
        const Vector2 world_point2 = cv_camera2.imageToWorld(Vector2(x2, y2));
        const Vector2 unprojected_point2 = virtual_camera2.worldToImage(world_point2);
        if (!cv_camera2.isInsideImage(unprojected_point2)) {
          match_scores(row, col) = 0;
          continue;
        }
        const FloatType depth2 = offscreen_renderer_->decodeDepthValue(depth_image2, unprojected_point2);
        if (depth2 > max_depth || depth2 <= min_depth) {
          match_scores(row, col) = 0;
          continue;
        }
        const Vector3 world_point_3d_2 = depth2 * world_point2.homogeneous();
        const Vector3 abs_world_point_3d_2 = pose2.getTransformationImageToWorld() * world_point_3d_2.homogeneous();
        if ((abs_world_point_3d_1 - abs_world_point_3d_2).squaredNorm() > max_projection_error_squared) {
          match_scores(row, col) = 0;
          continue;
        }
//        else if (std::abs(depth1 - depth2) > depth_tolerance) {
//          match_scores(row, col) = 0;
//          continue;
//        }
//        if ((image_point_2 - Vector2(x2, y2)).squaredNorm() >= max_reprojection_error_squared) {
//          match_scores(row, col) = 0;
//          continue;
//        }
//        cout << "Found match" << endl;
      }
    }
  }

#pragma GCC optimize("O3")
  std::vector<size_t> getMatchIndicesBasedOnLoweRatioTest(
          const SiftMatchMatrix& match_scores,
          const FloatType ratio_threshold = FloatType(0.8),
          const FloatType max_feature_distance = FloatType(0.7)) {
    std::vector<size_t> match_indices(match_scores.rows());
#pragma omp parallel for
    for (size_t row = 0; row < (size_t)match_scores.rows(); ++row) {
      size_t best_index = (size_t)-1;
      SiftScore best_score = std::numeric_limits<SiftScore>::min();
//      size_t second_best_index = (size_t)-1;
      SiftScore second_best_score = std::numeric_limits<SiftScore>::min();
      for (size_t col = 0; col < (size_t)match_scores.cols(); ++col) {
        const SiftScore score = match_scores(row, col);
        if (score > second_best_score) {
          if (score > best_score) {
            second_best_score = best_score;
//            second_best_index = best_index;
            best_score = score;
            best_index = col;
          }
          else {
            second_best_score = score;
//            second_best_index = col;
          }
        }
      }

      // SIFT descriptor vectors are normalized to length 512.
      const FloatType kDistNorm = FloatType(1.0 / (512.0 * 512.0));

      const FloatType best_distance_normed = std::acos(std::min(kDistNorm * best_score, FloatType(1.0)));
      // Check for minimum distance
      if (best_distance_normed > max_feature_distance) {
        best_index = (size_t)-1;
      }
      else {
        const FloatType second_best_distance_normed = std::acos(
                std::min(kDistNorm * second_best_score, FloatType(1.0)));
        if (best_distance_normed >= ratio_threshold * second_best_distance_normed) {
          best_index = (size_t) -1;
        }
      }
      match_indices[row] = best_index;
    }
    return match_indices;
  };

  std::vector<std::pair<size_t, size_t>> selectBestMatches(
          const SiftMatchMatrix& match_scores,
          const FloatType ratio_threshold = FloatType(0.8),
          const FloatType max_feature_distance = FloatType(0.7),
          const bool cross_check = true) {
    cout << "Selecting matches based on ratio test" << endl;
    std::vector<size_t> match_indices1 = getMatchIndicesBasedOnLoweRatioTest(
            match_scores, ratio_threshold, max_feature_distance);
    cout << "Matches #1 after ratio test: " << match_indices1.size() << endl;
    // Perform cross-check
    std::vector<std::pair<size_t, size_t>> matches;
    if (cross_check) {
      std::vector<size_t> match_indices2 = getMatchIndicesBasedOnLoweRatioTest(
              match_scores.transpose(), ratio_threshold, max_feature_distance);
      cout << "Matches #2 after ratio test: " << match_indices2.size() << endl;
      for (size_t i = 0; i < match_indices1.size(); ++i) {
        const size_t keypoint_index2 = match_indices1[i];
        if (keypoint_index2 == (size_t) -1) {
          continue;
        }
        if (match_indices2[keypoint_index2] == i) {
          matches.emplace_back(i, keypoint_index2);
        }
      }
    }
    else {
      for (size_t i = 0; i < match_indices1.size(); ++i) {
        const size_t keypoint_index2 = match_indices1[i];
        if (keypoint_index2 == (size_t) -1) {
          continue;
        }
        matches.emplace_back(i, keypoint_index2);
      }
    }
    cout << "Matches after cross-check: " << matches.size() << endl;
    return matches;
  }

  std::vector<Keypoint> undistortKeypoints(const OpenCVCameraType& camera, const std::vector<Keypoint>& keypoints) const {
    const OpenCVCameraType undist_camera(camera.width(), camera.height(), camera.intrinsics());
    std::vector<Keypoint> undist_keypoints;
    std::transform(keypoints.begin(), keypoints.end(), std::back_inserter(undist_keypoints),
                   [&](const Keypoint& keypoint) {
                     const Vector2 world_point = camera.imageToWorld(Vector2(keypoint.x(), keypoint.y()));
                     const Vector2 undist_image_point = undist_camera.worldToImage(world_point);
                     const Keypoint undist_keypoint(undist_image_point(0), undist_image_point(1), keypoint.scale(), keypoint.orientation());
                     return undist_keypoint;
                   });
    return undist_keypoints;
  }

  std::vector<Vector2> backprojectKeypoints(const OpenCVCameraType& camera, const std::vector<Keypoint>& keypoints) const {
    std::vector<Vector2> world_points;
    std::transform(keypoints.begin(), keypoints.end(), std::back_inserter(world_points),
                   [&](const Keypoint& keypoint) {
                     const Vector2 world_point = camera.imageToWorld(Vector2(keypoint.x(), keypoint.y()));
                     return world_point;
                   });
    return world_points;
  }

#pragma GCC optimize("O3")
  size_t colmapImageIdsToPairId(const ImageId image_id1, const ImageId image_id2) {
    if (image_id1 > image_id2) {
      return colmapImageIdsToPairId(image_id2, image_id1);
    }
    return (size_t)2147483647 * (size_t)image_id1 + (size_t)image_id2;
  }

  void exportInlierMatchesToColmap(const ImageId image_id1, const ImageId image_id2,
                                   const std::vector<std::pair<size_t, size_t>> match_indices) {
//    // Colmap Essential Matrix
//    const size_t config = 2;
    // Colmap Fundamental Matrix
    const size_t config = 3;

    const size_t pair_id = colmapImageIdsToPairId(image_id1, image_id2);
    const size_t blob_rows = match_indices.size();
    const size_t blob_cols = 2;
    std::vector<uint32_t> match_blob;
    match_blob.reserve(2 * match_indices.size());
    for (const auto &entry : match_indices) {
      match_blob.push_back((uint32_t) entry.first);
      match_blob.push_back((uint32_t) entry.second);
    }
    const string count_query = "SELECT COUNT(*) FROM inlier_matches WHERE pair_id = ?1";
    bh::SQLite3::Statement query_statement = sqlite_db_->prepare(count_query);
    query_statement.bindValue64(1, pair_id);
    const bh::SQLite3::RowResult query_result = sqlite_db_->executeSingle(query_statement);
    const bool record_exists = query_result.getInt(0) > 0;
    query_statement.finish();
    // Export matches to database
    auto bind_and_execute_values_lambda = [&](bh::SQLite3::Statement& statement) {
      statement.bindValue64(1, pair_id);
      statement.bindValue(2, blob_rows);
      statement.bindValue(3, blob_cols);
      statement.bindData(4, match_blob);
      statement.bindValue(5, config);
      sqlite_db_->executeWithoutResult(statement);
    };
    if (record_exists) {
      const string update_query = "UPDATE inlier_matches SET rows = ?2, cols = ?3, data = ?4, config = ?5 WHERE pair_id = ?1";
      bh::SQLite3::Statement statement = sqlite_db_->prepare(update_query);
      bind_and_execute_values_lambda(statement);
    }
    else {
      const string insert_query = "INSERT INTO inlier_matches (pair_id, rows, cols, data, config) VALUES (?1, ?2, ?3, ?4, ?5)";
      bh::SQLite3::Statement statement = sqlite_db_->prepare(insert_query);
      bind_and_execute_values_lambda(statement);
    }
  }

  void exportMatchesToColmap(const ImageId image_id1, const ImageId image_id2,
                             const std::vector<std::pair<size_t, size_t>> match_indices) {
    const size_t pair_id = colmapImageIdsToPairId(image_id1, image_id2);
    const size_t blob_rows = match_indices.size();
    const size_t blob_cols = 2;
    std::vector<uint32_t> match_blob;
    match_blob.reserve(2 * match_indices.size());
    for (const auto &entry : match_indices) {
      match_blob.push_back((uint32_t) entry.first);
      match_blob.push_back((uint32_t) entry.second);
    }
    const string count_query = "SELECT COUNT(*) FROM matches WHERE pair_id = ?1";
    bh::SQLite3::Statement query_statement = sqlite_db_->prepare(count_query);
    query_statement.bindValue64(1, pair_id);
    const bh::SQLite3::RowResult query_result = sqlite_db_->executeSingle(query_statement);
    const bool record_exists = query_result.getInt(0) > 0;
    query_statement.finish();
    // Export matches to database
    auto bind_and_execute_values_lambda = [&](bh::SQLite3::Statement& statement) {
      statement.bindValue64(1, pair_id);
      statement.bindValue(2, blob_rows);
      statement.bindValue(3, blob_cols);
      statement.bindData(4, match_blob);
      sqlite_db_->executeWithoutResult(statement);
    };
    if (record_exists) {
      const string update_query = "UPDATE matches SET rows = ?2, cols = ?3, data = ?4 WHERE pair_id = ?1";
      bh::SQLite3::Statement statement = sqlite_db_->prepare(update_query);
      bind_and_execute_values_lambda(statement);
    }
    else {
      const string insert_query = "INSERT INTO matches (pair_id, rows, cols, data) VALUES (?1, ?2, ?3, ?4)";
      bh::SQLite3::Statement statement = sqlite_db_->prepare(insert_query);
      bind_and_execute_values_lambda(statement);
    }
  }

  Vector3 triangulatePoint(const ProjectionMatrix& projection_matrix_left,
                           const ProjectionMatrix& projection_matrix_right,
                           const Vector2& left_point,
                           const Vector2& right_point) const {
    const ProjectionMatrix& p_mat_l = projection_matrix_left;
    const ProjectionMatrix& p_mat_r = projection_matrix_right;
    // Triangulation of stereo points according to ...
    // * R. Hartley and P. Sturm, Triangulation, Computer vision and image understanding 68.2, 1997.
    // * HZ, R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, Cambridge Univ. Press, 2003.
    const FloatType u1 = left_point(0);
    const FloatType v1 = left_point(1);
    const FloatType u2 = right_point(0);
    const FloatType v2 = right_point(1);
    Matrix4x4 A;
    A(0, 0) = u1 * p_mat_l(2, 0) - p_mat_l(0, 0);
    A(0, 1) = u1 * p_mat_l(2, 1) - p_mat_l(0, 1);
    A(0, 2) = u1 * p_mat_l(2, 2) - p_mat_l(0, 2);
    A(0, 3) = u1 * p_mat_l(2, 3) - p_mat_l(0, 3);
    A(1, 0) = v1 * p_mat_l(2, 0) - p_mat_l(1, 0);
    A(1, 1) = v1 * p_mat_l(2, 1) - p_mat_l(1, 1);
    A(1, 2) = v1 * p_mat_l(2, 2) - p_mat_l(1, 2);
    A(1, 3) = v1 * p_mat_l(2, 3) - p_mat_l(1, 3);
    A(2, 0) = u2 * p_mat_r(2, 0) - p_mat_r(0, 0);
    A(2, 1) = u2 * p_mat_r(2, 1) - p_mat_r(0, 1);
    A(2, 2) = u2 * p_mat_r(2, 2) - p_mat_r(0, 2);
    A(2, 3) = u2 * p_mat_r(2, 3) - p_mat_r(0, 3);
    A(3, 0) = v2 * p_mat_r(2, 0) - p_mat_r(1, 0);
    A(3, 1) = v2 * p_mat_r(2, 1) - p_mat_r(1, 1);
    A(3, 2) = v2 * p_mat_r(2, 2) - p_mat_r(1, 2);
    A(3, 3) = v2 * p_mat_r(2, 3) - p_mat_r(1, 3);
    Eigen::JacobiSVD<Matrix4x4> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    const Matrix4x4 U = svd.matrixU();
    const Matrix4x4 V = svd.matrixV();
//    const Vector4 S = svd.singularValues();
    const FloatType x = V(0, 3);
    const FloatType y = V(1, 3);
    const FloatType z = V(2, 3);
    const FloatType w = V(3, 3);
    const Vector3 point(x / w, y / w, z / w);
    return point;
  }

  std::pair<Matrix3x3, Vector3> decomposeEssentialMatrix(const EssentialMatrix& essential_matrix) const {
    // TODO: Conversion
    cv::Mat cv_essential_matrix = bh::opencv::convertEigenToCv32F(essential_matrix);
    cv::Mat cv_translation;
    cv::Mat cv_rotation1;
    cv::Mat cv_rotation2;
    cv::decomposeEssentialMat(cv_essential_matrix, cv_rotation1, cv_rotation2, cv_translation);
    const Matrix3x3 rotation1 = bh::opencv::convertCvToEigen<Matrix3x3>(cv_rotation1);
    const Matrix3x3 rotation2 = bh::opencv::convertCvToEigen<Matrix3x3>(cv_rotation2);
    const Vector3 translation = bh::opencv::convertCvToEigen<Vector3>(cv_translation);
    const PoseType left_pose(Vector3::Zero(), Quaternion::Identity());
    const ProjectionMatrix left_projection_matrix = left_pose.getTransformationWorldToImage4x4();
    const PoseType right_pose1(translation, rotation1);
    const PoseType right_pose2(-translation, rotation1);
    const PoseType right_pose3(translation, rotation2);
    const PoseType right_pose4(-translation, rotation2);
    const ProjectionMatrix right_projection_matrix1 = right_pose1.getTransformationWorldToImage4x4();
    const ProjectionMatrix right_projection_matrix2 = right_pose2.getTransformationWorldToImage4x4();
    const ProjectionMatrix right_projection_matrix3 = right_pose3.getTransformationWorldToImage4x4();
    const ProjectionMatrix right_projection_matrix4 = right_pose4.getTransformationWorldToImage4x4();
    const Vector2 left_point(0, 0);
    const Vector2 right_point(0, 0);
    const Vector3 point1 = triangulatePoint(left_projection_matrix, right_projection_matrix1, left_point, right_point);
    const Vector3 point2 = triangulatePoint(left_projection_matrix, right_projection_matrix2, left_point, right_point);
    const Vector3 point3 = triangulatePoint(left_projection_matrix, right_projection_matrix3, left_point, right_point);
    const Vector3 point4 = triangulatePoint(left_projection_matrix, right_projection_matrix4, left_point, right_point);
    const Vector3 proj_point1 = (left_projection_matrix * point1.homogeneous()).hnormalized();
    const Vector3 proj_point2 = (left_projection_matrix * point2.homogeneous()).hnormalized();
    const Vector3 proj_point3 = (left_projection_matrix * point3.homogeneous()).hnormalized();
    const Vector3 proj_point4 = (left_projection_matrix * point4.homogeneous()).hnormalized();
    if (proj_point1(2) > 0) {}
    // TODO: Triangulate point with all four solutions and check for which solution it is in front of both cameras
    return std::make_pair(rotation1, translation);
  };

  template <typename ModelT>
  std::pair<ModelT, std::vector<size_t>> ransacFindInliers(
          const size_t num_samples,
          const size_t num_samples_for_solver,
          const size_t num_iterations,
          std::function<std::pair<bool, ModelT>(const std::vector<size_t>&)> solver,
          std::function<bool(const ModelT&, const size_t sample_index)> inlier_predicate,
          std::function<bool(const ModelT&, const std::vector<size_t>& inlier_indices)> model_predicate) const {
    if (num_samples < num_samples_for_solver) {
      return std::make_pair(ModelT(), std::vector<size_t>());
    }
    std::mt19937_64 rnd;
    std::uniform_int_distribution<size_t> sample_index_distribution(0, num_samples - 1);
    ModelT best_model;
    std::vector<size_t> best_inlier_indices;
#pragma omp parallel for
    for (size_t i = 0; i < num_iterations; ++i) {
      // Build random sample set
      std::vector<size_t> random_sample_indices;
      random_sample_indices.reserve(num_samples_for_solver);
      while (random_sample_indices.size() < num_samples_for_solver) {
        const size_t sample_index = sample_index_distribution(rnd);
        const auto it = std::find(random_sample_indices.begin(), random_sample_indices.end(), sample_index);
        if (it == random_sample_indices.end()) {
          random_sample_indices.push_back(sample_index);
        }
      }
      bool model_valid;
      ModelT model;
      std::tie(model_valid, model) = solver(random_sample_indices);
      if (!model_valid) {
        continue;
      }
      std::vector<size_t> inlier_indices;
      for (size_t j = 0; j < num_samples; ++j) {
        if (inlier_predicate(model, j)) {
          inlier_indices.push_back(j);
        }
      }
      const size_t inlier_count = inlier_indices.size();
      if (inlier_count < num_samples_for_solver) {
        continue;
      }
      if (!model_predicate(model, inlier_indices)) {
        continue;
      }
#pragma omp critical
      if (inlier_count > best_inlier_indices.size()) {
        best_model = model;
        best_inlier_indices = inlier_indices;
      }
    }
    return std::make_pair(best_model, best_inlier_indices);
  }

#pragma GCC optimize("O0")
  bool run() {
    cout << "Reading mesh ..." << endl;
    MeshIOType::loadFromFile(options_.poisson_mesh_filename, poisson_mesh_);
    cout << "Number of vertices in input mesh: " << poisson_mesh_.m_Vertices.size() << endl;

    cout << "Reading features from Colmap ..." << endl;
    image_ids_ = getImageIdsFromColmap();
    image_names_ = getImageNamesFromColmap();
//    cout << "Image names: " << endl;
//    for (const auto& entry : image_names_) {
//      const ImageId& image_id = entry.first;
//      const string& image_name = entry.second;
//      cout << "  " << image_id << " -> " << image_name << endl;
//    }
    image_camera_ids_ = getImageCameraIdsFromColmap();

    cameras_ = getCamerasFromColmap();

    all_keypoints_ = getKeypointsFromColmap();
    all_descriptors_ = getDescriptorsFromColmap();

    // Undistort keypoints
    for (const auto& entry : all_keypoints_) {
      const ImageId image_id = entry.first;
      const std::vector<Keypoint>& keypoints = entry.second;
      const OpenCVCameraType &camera = cameras_.at(image_camera_ids_.at(image_id));
      std::vector<Keypoint> undist_keypoints = undistortKeypoints(camera, keypoints);
      all_undist_keypoints_.emplace(entry.first, std::move(undist_keypoints));
    }

    // Compute back-projected 3d points
    for (const auto& entry : all_keypoints_) {
      const ImageId image_id = entry.first;
      const std::vector<Keypoint>& keypoints = entry.second;
      const OpenCVCameraType &camera = cameras_.at(image_camera_ids_.at(image_id));
      std::vector<Vector2> world_points = backprojectKeypoints(camera, keypoints);
      all_world_points_.emplace(entry.first, std::move(world_points));
    }

    // Read prior image information
    prior_image_poses_ = readPriorImagePoses();

    // Create offscreen renderer
    virtual_camera_ = PinholeCamera::createSimple(
            options_.virtual_camera_width,
            options_.virtual_camera_height,
            options_.virtual_camera_focal_length);
    offscreen_renderer_.reset(new ViewpointOffscreenRenderer(virtual_camera_, &poisson_mesh_));

    // Debugging: Render mesh from prior image poses
    if (options_.dump_prior_mesh_images) {
      bh::boostfs::create_directory("dump");
      for (const auto &entry : prior_image_poses_) {
        const ImageId image_id = entry.first;
        const PoseType &pose = entry.second;
//        cout << "image_id=" << image_id << ", pose=" << pose << endl;
        QImage mesh_image = offscreen_renderer_->drawPoissonMesh(pose);
        const string mesh_image_filename = string("dump/mesh_image_") + std::to_string(image_id) + ".png";
        mesh_image.save(QString::fromStdString(mesh_image_filename));
        QImage mesh_depth_image = offscreen_renderer_->drawPoissonMeshDepth(pose);
        mesh_depth_image = offscreen_renderer_->convertEncodedDepthImageToRGB(
                mesh_depth_image, options_.cmap_min_depth, options_.cmap_max_depth);
        const string mesh_depth_image_filename = string("dump/mesh_depth_image_") + std::to_string(image_id) + ".png";
        mesh_depth_image.save(QString::fromStdString(mesh_depth_image_filename));
      }
    }

    if (options_.delete_feature_matches_in_db) {
      const string delete_query = "DELETE FROM matches";
      sqlite_db_->executeWithoutResult(delete_query);
    }

    boostfs::create_directory("dump");
    // Match features
    for (const ImageId image_id1 : image_ids_) {
//      if (image_id1 != 2) {
//        continue;
//      }
      // Render depth image
      const PoseType pose1 = prior_image_poses_.at(image_id1);
      const OpenCVCameraType cv_camera1 = cameras_.at(image_camera_ids_.at(image_id1));
      const PinholeCamera camera1(
              cv_camera1.width(), cv_camera1.height(), cv_camera1.intrinsics());
//      offscreen_renderer_->setCamera(camera1);
//      QImage mesh_image1 = offscreen_renderer_->drawPoissonMesh(pose1);
//      QImage depth_image1 = offscreen_renderer_->drawPoissonMeshDepth(pose1);
//      QImage depth_image1_cmap = offscreen_renderer_->convertEncodedDepthImageToRGB(
//              depth_image1, options_.cmap_min_depth, options_.cmap_max_depth);

      const std::vector<Keypoint>& keypoints1 = all_keypoints_.at(image_id1);
      const std::vector<Keypoint>& undist_keypoints1 = all_undist_keypoints_.at(image_id1);
      const std::vector<Vector2>& world_points1 = all_world_points_.at(image_id1);

      if (options_.dump_keypoint_images) {
        // Dump keypoints of image 1
        boostfs::path image_path1 = options_.images_path;
        image_path1 /= image_names_.at(image_id1);
        QImage keypoints_img1;
        keypoints_img1.load(QString::fromStdString(image_path1.string()));
        bh::vision::drawKeypoints(&keypoints_img1, keypoints1);
        const string keypoint_dump_filename = string("dump/keypoints_") + std::to_string(image_id1) + ".png";
        keypoints_img1.save(QString::fromStdString(keypoint_dump_filename));
      }

//      if (options_.dump_keypoint_images) {
//        // Dump keypoints of image 1 with mesh image
//        QImage mesh_keypoints_img1 = mesh_image1;
//        bh::vision::drawKeypoints(&mesh_keypoints_img1, undist_keypoints1);
//        const string mesh_keypoint_dump_filename1 = string("dump/keypoints_mesh_") + std::to_string(image_id1) + ".png";
//        mesh_keypoints_img1.save(QString::fromStdString(mesh_keypoint_dump_filename1));
//      }

      for (ImageId image_id2 : image_ids_) {
//        if (image_id2 != 4) {
//          continue;
//        }
        // No need to match each image twice
        if (image_id2 <= image_id1) {
          continue;
        }

        // Render depth image
        const PoseType pose2 = prior_image_poses_.at(image_id2);
        const PoseType pose1_inverse = pose1.inverse();
        const Vector3 prior_world_translation = pose1.getWorldPosition() - pose2.getWorldPosition();
        const Vector3 prior_translation = pose1.rotation().inverse() * prior_world_translation;
        const Vector3 normalized_prior_translation = prior_translation.normalized();
        const Quaternion prior_quaternion = pose2.quaternion() * pose1.quaternion().inverse();
        const Matrix3x3 prior_abs_rotation1 = pose1.rotation();
        const Matrix3x3 prior_abs_rotation2 = pose2.rotation();
        const Matrix3x3 prior_rotation = prior_quaternion.toRotationMatrix();
        const FloatType prior_angular_distance = (pose2.quaternion() * pose1.quaternion().inverse()).angularDistance(Quaternion::Identity());
        const FloatType prior_angular_distance2 = pose2.quaternion().angularDistance(pose1.quaternion());

        const OpenCVCameraType cv_camera2 = cameras_.at(image_camera_ids_.at(image_id2));
        const PinholeCamera camera2(
                cv_camera2.width(), cv_camera2.height(), cv_camera2.intrinsics());
//        offscreen_renderer_->setCamera(camera2);
//        QImage mesh_image2 = offscreen_renderer_->drawPoissonMesh(pose2);
//        QImage depth_image2 = offscreen_renderer_->drawPoissonMeshDepth(pose2);
//        QImage depth_image2_cmap = offscreen_renderer_->convertEncodedDepthImageToRGB(
//                depth_image2, options_.cmap_min_depth, options_.cmap_max_depth);

        const std::vector<Keypoint>& keypoints2 = all_keypoints_.at(image_id2);
        const std::vector<Keypoint>& undist_keypoints2 = all_undist_keypoints_.at(image_id2);
        const std::vector<Vector2>& world_points2 = all_world_points_.at(image_id2);

        cout << "Matching image " << image_id1 << " with " << image_id2 << endl;
//        BH_PRINT_VALUE(keypoints1.size());
//        BH_PRINT_VALUE(keypoints2.size());
        SiftMatchMatrix match_scores = computeMatchScores(
                all_descriptors_.at(image_id1), all_descriptors_.at(image_id2));

//        cout << "Invalidating matches based on minimum feature distance" << endl;
//        invalidateMatchesBasedOnFeatureDistance(
//                image_id1, image_id2, match_scores,
//                options_.max_feature_distance);
//        cout << "Invalidating matches based on geometry" << endl;
//        invalidateMatchesBasedOnPriorGeometry(
//                image_id1, image_id2, match_scores,
//                options_.min_depth, options_.max_depth, options_.max_projection_error);
//        for (size_t row = 0; row < (size_t)match_scores.rows(); ++row) {
//          const size_t match_count = match_scores.row(row).array().isFinite().count();
//          if (match_count > 0) {
//            cout << "Matches for descriptor " << row << ": " << match_count << endl;
//          }
//        }

        std::vector<std::pair<size_t, size_t>> matches = selectBestMatches(
                match_scores, options_.lowe_ratio_threshold, options_.max_feature_distance, options_.cross_check);

        exportMatchesToColmap(image_id1, image_id2, matches);



        {
          // Create keypoint-matches vector
          std::vector<KeypointMatch> keypoint_matches;
          for (const auto& entry : matches) {
            keypoint_matches.push_back(KeypointMatch(keypoints1[entry.first], keypoints2[entry.second]));
          }

          if (options_.dump_keypoint_images) {
            // Dump keypoints of image 2
            boostfs::path image_path2 = options_.images_path;
            image_path2 /= image_names_.at(image_id2);
            QImage keypoints_img2;
            keypoints_img2.load(QString::fromStdString(image_path2.string()));
            bh::vision::drawKeypoints(&keypoints_img2, keypoints2);
            const string keypoint_dump_filename2 = string("dump/raw_keypoints_") + std::to_string(image_id2) + ".png";
            keypoints_img2.save(QString::fromStdString(keypoint_dump_filename2));
          }

          if (options_.dump_match_images) {
            // Dump keypoint matches
            boostfs::path image_path1 = options_.images_path;
            image_path1 /= image_names_.at(image_id1);
            boostfs::path image_path2 = options_.images_path;
            image_path2 /= image_names_.at(image_id2);
            QImage keypoints_img1;
            keypoints_img1.load(QString::fromStdString(image_path1.string()));
            QImage keypoints_img2;
            keypoints_img2.load(QString::fromStdString(image_path2.string()));
            QImage match_img = bh::vision::drawKeypointsAndMatches(keypoints_img1, keypoints_img2, keypoint_matches);
            const string match_dump_filename = string("dump/raw_matches_") +
                                               std::to_string(image_id1) + "_" + std::to_string(image_id2) + ".png";
            match_img.save(QString::fromStdString(match_dump_filename));
          }
        }

        std::vector<std::pair<size_t, size_t>> inlier_matches;
        if (matches.size() > 0) {
          // Perform RANSAC to find inlier set
          const size_t num_samples = matches.size();
          const size_t num_samples_for_solver = 8;
          const size_t num_ransac_iterations = options_.num_ransac_iterations;
          const size_t min_num_inliers = std::max(options_.min_num_inliers, num_samples_for_solver);
//          const FloatType epipolar_constraint_threshold = FloatType(0.01);
          const FloatType sampson_distance_threshold = options_.sampson_distance_threshold;

          auto fundamental_matrix_solver_8point
                  = [&](const std::vector<size_t> &sample_indices) -> std::pair<bool, FundamentalMatrix> {
            std::vector<Vector2> points1;
            std::vector<Vector2> points2;
            for (size_t sample_index : sample_indices) {
              size_t index1;
              size_t index2;
              std::tie(index1, index2) = matches[sample_index];
              const Keypoint keypoint1 = undist_keypoints1[index1];
              const Keypoint keypoint2 = undist_keypoints2[index2];
              points1.push_back(Vector2(keypoint1.x(), keypoint1.y()));
              points2.push_back(Vector2(keypoint2.x(), keypoint2.y()));
            }
            bool fundamental_matrix_valid;
            FundamentalMatrix fundamental_matrix;
            std::tie(fundamental_matrix_valid, fundamental_matrix)
                    = bh::vision::computeFundamentalMatrix8Point<FloatType>(points1, points2);
            return std::make_pair(fundamental_matrix_valid, fundamental_matrix);
          };

          auto fundamental_matrix_solver
                  = [&](const std::vector<size_t> &sample_indices) -> std::pair<bool, FundamentalMatrix> {
                    std::vector<Vector2> points1;
                    std::vector<Vector2> points2;
                    for (size_t sample_index : sample_indices) {
                      size_t index1;
                      size_t index2;
                      std::tie(index1, index2) = matches[sample_index];
                      const Keypoint keypoint1 = undist_keypoints1[index1];
                      const Keypoint keypoint2 = undist_keypoints2[index2];
                      points1.push_back(Vector2(keypoint1.x(), keypoint1.y()));
                      points2.push_back(Vector2(keypoint2.x(), keypoint2.y()));
                    }
                    bool fundamental_matrix_valid;
                    FundamentalMatrix fundamental_matrix;
                    std::tie(fundamental_matrix_valid, fundamental_matrix)
                            = bh::vision::computeFundamentalMatrix<FloatType>(points1, points2);
                    return std::make_pair(fundamental_matrix_valid, fundamental_matrix);
                  };

          auto fundamental_matrix_inlier_predicate = [&](const FundamentalMatrix &fundamental_matrix, const size_t sample_index) {
            size_t index1;
            size_t index2;
            std::tie(index1, index2) = matches[sample_index];
            const Keypoint keypoint1 = undist_keypoints1[index1];
            const Keypoint keypoint2 = undist_keypoints2[index2];
            const Vector2 point1(keypoint1.x(), keypoint1.y());
            const Vector2 point2(keypoint2.x(), keypoint2.y());
            const FloatType sampson_distance = bh::vision::computeSampsonDistance(point1, point2, fundamental_matrix);
//            const Vector3 product_right = fundamental_matrix * point2.homogeneous();
//            const FloatType dot_product = point1.homogeneous().dot(product_right);
            if (sampson_distance <= sampson_distance_threshold) {
//            if (std::abs(dot_product) <= epipolar_constraint_threshold) {
              return true;
            }
            else {
              return false;
            }
          };

          const FloatType max_angular_distance_to_prior = options_.max_angular_distance_to_prior_degrees * M_PI / FloatType(180.0);

          auto fundamental_matrix_model_predicate = [&](const FundamentalMatrix &fundamental_matrix,
                                                        const std::vector<size_t>& inlier_indices) {
            return true;
            BH_ASSERT(inlier_indices.size() > 0);
            const EssentialMatrix essential_matrix = bh::vision::essentialMatrixFromFundamentalMatrix(
                    fundamental_matrix, cv_camera1, cv_camera2);
            size_t index1;
            size_t index2;
            std::tie(index1, index2) = matches[inlier_indices[0]];
            const Vector2 point1 = world_points1[index1];
            const Vector2 point2 = world_points2[index2];
            bool decompose_essential_matrix_success;
            SE3Transform se3_transform;
            std::tie(decompose_essential_matrix_success, se3_transform) = bh::vision::decomposeEssentialMatrix(
                    essential_matrix, point1, point2);
            if (!decompose_essential_matrix_success) {
              return false;
            }
            const SE3Transform right_to_left_se3_transform = se3_transform.inverse();
            const Matrix3x3 rotation = right_to_left_se3_transform.rotation();
            const FloatType angular_distance_to_prior
                    = right_to_left_se3_transform.quaternion().angularDistance(prior_quaternion);
            if (angular_distance_to_prior > max_angular_distance_to_prior) {
              return false;
            }
            const Vector3 normalized_estimated_translation = right_to_left_se3_transform.translation().normalized();
            const FloatType dot_product = normalized_estimated_translation.dot(normalized_prior_translation);
            if (dot_product < options_.min_translation_dot_product_with_prior) {
              return false;
            }
            return true;
          };

//          auto essential_matrix_solver_8point
//                  = [&](const std::vector<size_t> &sample_indices) -> std::pair<bool, EssentialMatrix> {
//                    std::vector<Vector2> points1;
//                    std::vector<Vector2> points2;
//                    for (size_t sample_index : sample_indices) {
//                      size_t index1;
//                      size_t index2;
//                      std::tie(index1, index2) = matches[sample_index];
//                      points1.push_back(world_points1[index1]);
//                      points2.push_back(world_points1[index2]);
//                    }
//                    bool essential_matrix_valid;
//                    EssentialMatrix essential_matrix;
//                    std::tie(essential_matrix_valid, essential_matrix)
//                            = bh::vision::computeFundamentalMatrix8Point<FloatType>(points1, points2);
//                    return std::make_pair(essential_matrix_valid, essential_matrix);
//                  };
//
//          auto essential_matrix_solver
//                  = [&](const std::vector<size_t> &sample_indices) -> std::pair<bool, EssentialMatrix> {
//                    std::vector<Vector2> points1;
//                    std::vector<Vector2> points2;
//                    for (size_t sample_index : sample_indices) {
//                      size_t index1;
//                      size_t index2;
//                      std::tie(index1, index2) = matches[sample_index];
//                      points1.push_back(world_points1[index1]);
//                      points2.push_back(world_points1[index2]);
//                    }
//                    bool essential_matrix_valid;
//                    EssentialMatrix essential_matrix;
//                    std::tie(essential_matrix_valid, essential_matrix)
//                            = bh::vision::computeFundamentalMatrix<FloatType>(points1, points2);
//                    return std::make_pair(essential_matrix_valid, essential_matrix);
//                  };
//
//          auto essential_matrix_inlier_predicate = [&](const EssentialMatrix &essential_matrix, const size_t sample_index) {
//            size_t index1;
//            size_t index2;
//            std::tie(index1, index2) = matches[sample_index];
//            const Vector2 point1 = world_points1[index1];
//            const Vector2 point2 = world_points2[index2];
//            const FloatType sampson_distance = bh::vision::computeSampsonDistance(point1, point2, essential_matrix);
////            const Vector3 product_right = fundamental_matrix * point2.homogeneous();
////            const FloatType dot_product = point1.homogeneous().dot(product_right);
//            if (sampson_distance <= sampson_distance_threshold) {
////            if (std::abs(dot_product) <= epipolar_constraint_threshold) {
//              return true;
//            }
//            else {
//              return false;
//            }
//          };
//
//          auto essential_matrix_model_predicate = [&](const EssentialMatrix &essential_matrix,
//                                                      const std::vector<size_t>& inlier_indices) {
//            BH_ASSERT(inlier_indices.size() > 0);
//            size_t index1;
//            size_t index2;
//            std::tie(index1, index2) = matches[inlier_indices[0]];
//            const Vector2 point1 = world_points1[index1];
//            const Vector2 point2 = world_points2[index2];
//            bool decompose_essential_matrix_success;
//            SE3Transform se3_transform;
//            std::tie(decompose_essential_matrix_success, se3_transform) = bh::vision::decomposeEssentialMatrix(
//                    essential_matrix, point1, point2);
//            if (!decompose_essential_matrix_success) {
//              return false;
//            }
//            const SE3Transform right_to_left_se3_transform = se3_transform.inverse();
//            const Matrix3x3 rotation = right_to_left_se3_transform.rotation();
//            const FloatType angular_distance_to_prior
//                    = right_to_left_se3_transform.quaternion().angularDistance(prior_quaternion);
//            if (angular_distance_to_prior > max_angular_distance_to_prior) {
//              return false;
//            }
//            const Vector3 normalized_estimated_translation = right_to_left_se3_transform.translation().normalized();
//            const FloatType dot_product = normalized_estimated_translation.dot(normalized_prior_translation);
//            if (dot_product < options_.min_translation_dot_product_with_prior) {
//              return false;
//            }
//            return true;
//          };

          std::vector<size_t> inlier_indices;
//          FundamentalMatrix fundamental_matrix_ransac;
//          std::tie(fundamental_matrix_ransac, inlier_indices) = ransacFindInliers<FundamentalMatrix>(
//                  num_samples,
//                  num_samples_for_solver,
//                  num_ransac_iterations,
//                  fundamental_matrix_solver_8point,
//                  fundamental_matrix_inlier_predicate,
//                  fundamental_matrix_model_predicate
//          );
          FundamentalMatrix fundamental_matrix_ransac;
          std::tie(fundamental_matrix_ransac, inlier_indices) = ransacFindInliers<FundamentalMatrix>(
                  num_samples,
                  num_samples_for_solver,
                  num_ransac_iterations,
                  fundamental_matrix_solver_8point,
                  fundamental_matrix_inlier_predicate,
                  fundamental_matrix_model_predicate
          );
          if (inlier_indices.size() >= min_num_inliers) {

            // TODO
            exportInlierMatchesToColmap(image_id1, image_id2, inlier_matches);

            if (options_.dump_match_images) {
              std::vector<KeypointMatch> tmp_keypoint_matches;
              for (size_t inlier_index : inlier_indices) {
                size_t index1;
                size_t index2;
                std::tie(index1, index2) = matches[inlier_index];
                const Keypoint keypoint1 = undist_keypoints1[index1];
                const Keypoint keypoint2 = undist_keypoints2[index2];
                tmp_keypoint_matches.push_back(KeypointMatch(keypoint1, keypoint2));
              }
              // Dump keypoint matches
              boostfs::path image_path1 = options_.images_path;
              image_path1 /= image_names_.at(image_id1);
              boostfs::path image_path2 = options_.images_path;
              image_path2 /= image_names_.at(image_id2);
              QImage keypoints_img1;
              keypoints_img1.load(QString::fromStdString(image_path1.string()));
              QImage keypoints_img2;
              keypoints_img2.load(QString::fromStdString(image_path2.string()));
              QImage match_img = bh::vision::drawKeypointsAndMatches(keypoints_img1, keypoints_img2, tmp_keypoint_matches);
              const string match_dump_filename = string("dump/inlier_matches_") +
                                                 std::to_string(image_id1) + "_" + std::to_string(image_id2) + ".png";
              match_img.save(QString::fromStdString(match_dump_filename));
            }

            if (options_.dump_epipolar_lines) {
              // Dump epipolar lines of matches
              std::vector<Keypoint> tmp_keypoints1;
              std::vector<Keypoint> tmp_keypoints2;
              for (size_t inlier_index : inlier_indices) {
                size_t index1;
                size_t index2;
                std::tie(index1, index2) = matches[inlier_index];
                const Keypoint keypoint1 = undist_keypoints1[index1];
                const Keypoint keypoint2 = undist_keypoints2[index2];
                tmp_keypoints1.push_back(keypoint1);
                tmp_keypoints2.push_back(keypoint2);
              }
              boostfs::path image_path1 = options_.images_path;
              image_path1 /= image_names_.at(image_id1);
              boostfs::path image_path2 = options_.images_path;
              image_path2 /= image_names_.at(image_id2);
              QImage keypoints_img1;
              keypoints_img1.load(QString::fromStdString(image_path1.string()));
              bh::vision::drawKeypoints(&keypoints_img1, tmp_keypoints1);
              QImage keypoints_img2;
              keypoints_img2.load(QString::fromStdString(image_path2.string()));
              bh::vision::drawKeypoints(&keypoints_img2, tmp_keypoints2);
//              const FundamentalMatrix fundamental_matrix = bh::vision::fundamentalMatrixFromEssentialMatrix(
//                      fundamental_matrix_ransac, cv_camera1, cv_camera2);
              QImage epipolar_lines_img = bh::vision::drawKeypointsAndEpipolarLines(
                      keypoints_img1, keypoints_img2, fundamental_matrix_ransac, tmp_keypoints1, tmp_keypoints2);
              const string epipolar_lines_dump_filename = string("dump/inlier_epipolar_lines_") +
                                                          std::to_string(image_id1) + "_" + std::to_string(image_id2) + ".png";
              epipolar_lines_img.save(QString::fromStdString(epipolar_lines_dump_filename));
            }

//            bool model_valid = true;
//            FundamentalMatrix fundamental_matrix = fundamental_matrix_ransac;
            bool model_valid;
            FundamentalMatrix fundamental_matrix;
            std::tie(model_valid, fundamental_matrix) = fundamental_matrix_solver(inlier_indices);
            if (model_valid) {
              for (size_t inlier_index : inlier_indices) {
                inlier_matches.push_back(matches[inlier_index]);
              }
            }
            const bool fundamental_matrix_valid = inlier_matches.size() >= num_samples_for_solver;
            if (fundamental_matrix_valid) {
              BH_ASSERT(inlier_indices.size() > 0);
              const EssentialMatrix essential_matrix = bh::vision::essentialMatrixFromFundamentalMatrix(
                      fundamental_matrix, cv_camera1, cv_camera2);
              BH_PRINT_VALUE(fundamental_matrix_model_predicate(fundamental_matrix_ransac, inlier_indices));
              BH_PRINT_VALUE(fundamental_matrix_model_predicate(fundamental_matrix, inlier_indices));
              size_t index1;
              size_t index2;
              std::tie(index1, index2) = matches[inlier_indices[0]];
              const Vector2 point1 = world_points1[index1];
              const Vector2 point2 = world_points2[index2];
              bool decompose_essential_matrix_success;
              SE3Transform se3_transform;
              std::tie(decompose_essential_matrix_success, se3_transform) = bh::vision::decomposeEssentialMatrix(
                      essential_matrix, point1, point2);
              if (decompose_essential_matrix_success) {
                const SE3Transform right_to_left_se3_transform = se3_transform.inverse();
                const Matrix3x3 rotation = right_to_left_se3_transform.rotation();
                const Matrix3x3 rotation2 = right_to_left_se3_transform.rotation().inverse();
                const FloatType angular_distance_to_prior
                        = right_to_left_se3_transform.quaternion().angularDistance(prior_quaternion);
                const Vector3 normalized_estimated_translation = right_to_left_se3_transform.translation().normalized();
                const FloatType angular_distance = right_to_left_se3_transform.quaternion().angularDistance(
                        Quaternion::Identity());
                const FloatType dot_product = normalized_estimated_translation.dot(normalized_prior_translation);
                BH_PRINT_VALUE(angular_distance_to_prior * 180 / M_PI);
                BH_PRINT_VALUE(dot_product);
                BH_PRINT_VALUE(std::acos(dot_product) * 180 / M_PI);
                BH_PRINT_VALUE(fundamental_matrix_ransac);
                BH_PRINT_VALUE(fundamental_matrix);
                BH_ASSERT(fundamental_matrix_model_predicate(fundamental_matrix_ransac, inlier_indices));
              }

              // TODO
              exportInlierMatchesToColmap(image_id1, image_id2, inlier_matches);

              cout << "Inlier matches: " << inlier_matches.size() << endl;

              if (options_.guided_matching) {
                cout << "Performing guided matching of image " << image_id1 << " with " << image_id2 << endl;
                SiftMatchMatrix guided_match_scores = computeMatchScoresGuided(
                        all_descriptors_.at(image_id1), all_descriptors_.at(image_id2),
                        [&](const size_t index1, const size_t index2) {
                          const Keypoint keypoint1 = undist_keypoints1[index1];
                          const Keypoint keypoint2 = undist_keypoints2[index2];
                          const Vector2 point1(keypoint1.x(), keypoint1.y());
                          const Vector2 point2(keypoint2.x(), keypoint2.y());
                          const FloatType sampson_distance
                                  = bh::vision::computeSampsonDistance(point1, point2, fundamental_matrix);
                          const bool accept = sampson_distance <= options_.max_epipolar_error * options_.max_epipolar_error;
                          //                        if (accept && std::abs(image_point1(1) - image_point2(1) > 50)) {
                          //                          BH_PRINT_VALUE((fundamental_matrix * image_point1.homogeneous()).transpose());
                          //                          BH_PRINT_VALUE((image_point2.homogeneous().transpose() * fundamental_matrix));
                          //                          BH_PRINT_VALUE(fundamental_matrix);
                          //                        }
                          return accept;
                        });

                inlier_matches = selectBestMatches(
                        guided_match_scores,
                        options_.lowe_ratio_threshold,
                        options_.max_feature_distance,
                        options_.cross_check);

                cout << "Guided matches: " << inlier_matches.size() << endl;

                if (options_.dump_match_images) {
                  std::vector<KeypointMatch> tmp_keypoint_matches;
                  for (const auto& inlier_match : inlier_matches) {
                    const Keypoint keypoint1 = undist_keypoints1[inlier_match.first];
                    const Keypoint keypoint2 = undist_keypoints2[inlier_match.second];
                    tmp_keypoint_matches.push_back(KeypointMatch(keypoint1, keypoint2));
                  }
                  // Dump keypoint matches
                  boostfs::path image_path1 = options_.images_path;
                  image_path1 /= image_names_.at(image_id1);
                  boostfs::path image_path2 = options_.images_path;
                  image_path2 /= image_names_.at(image_id2);
                  QImage keypoints_img1;
                  keypoints_img1.load(QString::fromStdString(image_path1.string()));
                  QImage keypoints_img2;
                  keypoints_img2.load(QString::fromStdString(image_path2.string()));
                  QImage match_img = bh::vision::drawKeypointsAndMatches(keypoints_img1, keypoints_img2, tmp_keypoint_matches);
                  const string match_dump_filename = string("dump/guided_matches_") +
                                                     std::to_string(image_id1) + "_" + std::to_string(image_id2) + ".png";
                  match_img.save(QString::fromStdString(match_dump_filename));
                }

                if (options_.dump_epipolar_lines) {
                  // Dump epipolar lines of matches
                  std::vector<Keypoint> tmp_keypoints1;
                  std::vector<Keypoint> tmp_keypoints2;
                  for (const auto& inlier_match : inlier_matches) {
                    const Keypoint keypoint1 = undist_keypoints1[inlier_match.first];
                    const Keypoint keypoint2 = undist_keypoints2[inlier_match.second];
                    tmp_keypoints1.push_back(keypoint1);
                    tmp_keypoints2.push_back(keypoint2);
                  }
                  boostfs::path image_path1 = options_.images_path;
                  image_path1 /= image_names_.at(image_id1);
                  boostfs::path image_path2 = options_.images_path;
                  image_path2 /= image_names_.at(image_id2);
                  QImage keypoints_img1;
                  keypoints_img1.load(QString::fromStdString(image_path1.string()));
                  QImage keypoints_img2;
                  keypoints_img2.load(QString::fromStdString(image_path2.string()));
//                const FundamentalMatrix fundamental_matrix = bh::vision::fundamentalMatrixFromEssentialMatrix(
//                        essential_matrix_ransac, cv_camera1, cv_camera2);
                  QImage epipolar_lines_img = bh::vision::drawKeypointsAndEpipolarLines(
                          keypoints_img1, keypoints_img2, fundamental_matrix, tmp_keypoints1, tmp_keypoints2);
                  const string epipolar_lines_dump_filename = string("dump/guided_epipolar_lines_") +
                                                              std::to_string(image_id1) + "_" + std::to_string(image_id2) + ".png";
                  epipolar_lines_img.save(QString::fromStdString(epipolar_lines_dump_filename));
                }
              }

            }
            else {
              std::cout << "Could not find suitable model" << std::endl;
              inlier_matches.clear();
            }
          }
        }

        // Create keypoint-matches vector
        std::vector<KeypointMatch> keypoint_matches;
        std::vector<KeypointMatch> undist_keypoint_matches;
        for (const auto& entry : inlier_matches) {
          keypoint_matches.push_back(KeypointMatch(keypoints1[entry.first], keypoints2[entry.second]));
          undist_keypoint_matches.push_back(KeypointMatch(undist_keypoints1[entry.first], undist_keypoints2[entry.second]));
        }
//        // Create undistorted keypoint-matches
//        std::vector<KeypointMatch> undist_keypoint_matches;
//        for (const KeypointMatch& keypoint_match : keypoint_matches) {
//          // Undistort keypoint 1
//          const Keypoint& keypoint1 = keypoint_match.keypoint1();
//          const Vector2 world_point1 = cv_camera1.imageToWorld(Vector2(keypoint1.x(), keypoint1.y()));
//          const Vector2 undist_image_point1 = virtual_camera1.worldToImage(world_point1);
//          const Keypoint undist_keypoint1(undist_image_point1(0), undist_image_point1(1), keypoint1.scale(), keypoint1.orientation());
//          // Undistort keypoint 2
//          const Keypoint& keypoint2 = keypoint_match.keypoint2();
//          const Vector2 world_point2 = cv_camera2.imageToWorld(Vector2(keypoint2.x(), keypoint2.y()));
//          const Vector2 undist_image_point2 = virtual_camera2.worldToImage(world_point2);
//          const Keypoint undist_keypoint2(undist_image_point2(0), undist_image_point2(1), keypoint2.scale(), keypoint2.orientation());
//          // Add undistorted keypoint match
//          const KeypointMatch undist_keypoint_match(undist_keypoint1, undist_keypoint2);
//          undist_keypoint_matches.push_back(undist_keypoint_match);
//
////          cout << "keypoint1=(" << keypoint1.x() << ", " << keypoint1.y() << ")" << endl;
////          cout << "keypoint2=(" << keypoint2.x() << ", " << keypoint2.y() << ")" << endl;
////          cout << "undist_keypoint1=(" << undist_keypoint1.x() << ", " << undist_keypoint1.y() << ")" << endl;
////          cout << "undist_keypoint2=(" << undist_keypoint2.x() << ", " << undist_keypoint2.y() << ")" << endl;
////          const FloatType depth1 = offscreen_renderer_->decodeDepthValue(depth_image1, undist_image_point1);
////          const FloatType depth2 = offscreen_renderer_->decodeDepthValue(depth_image2, undist_image_point2);
////          cout << "depth1=" << depth1 << endl;
////          cout << "depth2=" << depth2 << endl;
//        }

        exportInlierMatchesToColmap(image_id1, image_id2, inlier_matches);

        if (options_.dump_keypoint_images) {
          // Dump keypoints of image 2
          boostfs::path image_path2 = options_.images_path;
          image_path2 /= image_names_.at(image_id2);
          QImage keypoints_img2;
          keypoints_img2.load(QString::fromStdString(image_path2.string()));
          bh::vision::drawKeypoints(&keypoints_img2, keypoints2);
          const string keypoint_dump_filename2 = string("dump/keypoints_") + std::to_string(image_id2) + ".png";
          keypoints_img2.save(QString::fromStdString(keypoint_dump_filename2));
        }

//        if (options_.dump_keypoint_images) {
//          // Dump keypoints of image 2 with mesh image
//          QImage mesh_keypoints_img2 = mesh_image2;
//          bh::vision::drawKeypoints(&mesh_keypoints_img2, undist_keypoints2);
//          const string mesh_keypoint_dump_filename2 =
//                  string("dump/keypoints_mesh_") + std::to_string(image_id2) + ".png";
//          mesh_keypoints_img2.save(QString::fromStdString(mesh_keypoint_dump_filename2));
//        }

//        if (options_.dump_match_images) {
//          // Dump keypoint matches with mesh images
//          QImage mesh_match_img = bh::vision::drawKeypointsAndMatches(mesh_image1, mesh_image2,
//                                                                      undist_keypoint_matches);
//          const string mesh_match_dump_filename = string("dump/matches_mesh_") +
//                                                  std::to_string(image_id1) + "_" + std::to_string(image_id2) + ".png";
//          mesh_match_img.save(QString::fromStdString(mesh_match_dump_filename));
//          // Dump keypoint matches with mesh depth images
//          QImage depth_match_img = bh::vision::drawKeypointsAndMatches(depth_image1_cmap, depth_image2_cmap,
//                                                                       undist_keypoint_matches);
//          const string depth_match_dump_filename = string("dump/matches_depth_") +
//                                                   std::to_string(image_id1) + "_" + std::to_string(image_id2) + ".png";
//          depth_match_img.save(QString::fromStdString(depth_match_dump_filename));
//        }
      }
    }

    // Dump images with feature matches

    return true;
  }

private:
  Options options_;
  PinholeCamera virtual_camera_;
  MeshDataType poisson_mesh_;
  std::unique_ptr<bh::SQLite3> sqlite_db_;
  std::vector<int> image_ids_;
  std::unordered_map<ImageId, string> image_names_;
  std::unordered_map<ImageId, CameraId> image_camera_ids_;
  CameraIdToOpenCVCameraMap cameras_;
  std::unordered_map<ImageId, std::vector<Keypoint>> all_keypoints_;
  std::unordered_map<ImageId, std::vector<Keypoint>> all_undist_keypoints_;
  std::unordered_map<ImageId, std::vector<Vector2>> all_world_points_;
  ImageIdToSiftDescriptorVectorMap all_descriptors_;
  ImageIdToPoseMap prior_image_poses_;
  std::unique_ptr<ViewpointOffscreenRenderer> offscreen_renderer_;
};

const string ImageMatcherCmdline::Options::kPrefix = "image_matcher";

std::pair<bool, boost::program_options::variables_map> processOptions(
        int argc, char** argv, std::map<string, std::unique_ptr<bh::ConfigOptions>>& config_options)
{
  namespace po = boost::program_options;

  po::variables_map vm;
  try {
    po::options_description generic_options("Generic options");
    generic_options.add_options()
            ("help", "Produce help message")
            ("config-file", po::value<string>()->default_value("image_matcher.cfg"), "Config file.")
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
#if WITH_OPENGL_OFFSCREEN
  QApplication qapp(argc, argv);
  qapp.setApplicationName("Quad3DR viewpoint planner image matcher");
#endif

  std::map<std::string, std::unique_ptr<bh::ConfigOptions>> config_options =
          ImageMatcherCmdline::getConfigOptions();

  // Handle command line and config file
  std::pair<bool, boost::program_options::variables_map> cmdline_result =
          processOptions(argc, argv, config_options);
  if (!cmdline_result.first) {
    return 1;
  }
  boost::program_options::variables_map vm = std::move(cmdline_result.second);

  ImageMatcherCmdline image_matcher_cmdline(config_options);

  if (image_matcher_cmdline.run()) {
    return 0;
  }
  else {
    return -1;
  }
}
