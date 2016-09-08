#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

#include <tclap/CmdLine.h>
#include <video_source_zed.h>

// TODO
const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   calibration  -w=4 -h=5 -s=0.025 -o=camera.yml -op -oe\n"
" \n"
" example command line for calibration from a list of stored images:\n"
"   imagelist_creator image_list.xml *.png\n"
"   calibration -w=4 -h=5 -s=0.025 -o=camera.yml -op -oe image_list.xml\n"
" where image_list.xml is the standard OpenCV XML/YAML\n"
" use imagelist_creator to create the xml or yaml list\n"
" file consisting of the list of strings, e.g.:\n"
" \n"
"<?xml version=\"1.0\"?>\n"
"<opencv_storage>\n"
"<images>\n"
"view000.png\n"
"view001.png\n"
"<!-- view002.png -->\n"
"view003.png\n"
"view010.png\n"
"one_extra_view.jpg\n"
"</images>\n"
"</opencv_storage>\n";




// TODO
const char* liveCaptureHelp =
    "When the live video from camera is used as input, the following hot-keys may be used:\n"
        "  <ESC>, 'q' - quit the program\n"
        "  'c' - keep frame\n"
        "  'u' - switch undistortion on/off\n";

// TODO
static void help()
{
    printf("This is a camera calibration sample.\n"
        "Usage: calibration\n"
        "     -w=<board_width>         # the number of inner corners per one of board dimension\n"
        "     -h=<board_height>        # the number of inner corners per another board dimension\n"
        "     [-pt=<pattern>]          # the type of pattern: chessboard or circles' grid\n"
        "     [-n=<number_of_frames>]  # the number of frames to use for calibration\n"
        "                              # (if not specified, it will be set to the number\n"
        "                              #  of board views actually available)\n"
        "     [-d=<delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
        "                              # (used only for video capturing)\n"
        "     [-s=<squareSize>]       # square size in some user-defined units (1 by default)\n"
        "     [-o=<out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
        "     [-op]                    # write detected feature points\n"
        "     [-oe]                    # write extrinsic parameters\n"
        "     [-zt]                    # assume zero tangential distortion\n"
        "     [-a=<aspectRatio>]      # fix aspect ratio (fx/fy)\n"
        "     [-p]                     # fix the principal point at the center\n"
        "     [-v]                     # flip the captured images around the horizontal axis\n"
        "     [-V]                     # use a video file, and not an image list, uses\n"
        "                              # [input_data] string for the video file name\n"
        "     [-su]                    # show undistorted images after calibration\n"
        "     [input_data]             # input data, one of the following:\n"
        "                              #  - text file with a list of the images of the board\n"
        "                              #    the text file can be generated with imagelist_creator\n"
        "                              #  - name of video file with a video of the board\n"
        "                              # if input_data not specified, a live view from the camera is used\n"
        "\n");
    printf("\n%s",usage);
    printf("\n%s", liveCaptureHelp);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static bool readStringListOpenCV(const std::string &filename, std::vector<std::string> &l)
{
    l.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if(!fs.isOpened())
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != cv::FileNode::SEQ )
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
        l.push_back((std::string)*it);
    return true;
}

static std::vector<std::string> readStringList(const std::string &filename)
{
  std::vector<std::string> list;
  std::ifstream in(filename);
  if (!in.is_open())
  {
    throw std::runtime_error("Unable to open list file");
  }
  std::string line;
  while (std::getline(in, line))
  {
    list.push_back(line);
  }
  return list;
}

static double computeReprojectionErrors(
        const std::vector<std::vector<cv::Point3f>> &object_points,
        const std::vector<std::vector<cv::Point2f>> &image_points,
        const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
        const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
        std::vector<float> &per_view_errors)
{
  std::vector<cv::Point2f> image_points_2;
  int total_points = 0;
  double total_err = 0, err;
  per_view_errors.resize(object_points.size());

  for (int i = 0; i < (int)object_points.size(); i++)
  {
    cv::projectPoints(
      cv::Mat(object_points[i]),
      rvecs[i], tvecs[i],
      camera_matrix, dist_coeffs, image_points_2
    );
    err = norm(cv::Mat(image_points[i]), cv::Mat(image_points_2), cv::NORM_L2);
    int n = (int)object_points[i].size();
    per_view_errors[i] = (float)std::sqrt(err*err/n);
    total_err += err*err;
    total_points += n;
  }

  return std::sqrt(total_err/total_points);
}

static void calculateChessboardCorners(
    const cv::Size &board_size, float square_size, Pattern pattern_type,
    std::vector<cv::Point3f> &corners)
{
  corners.resize(0);

  switch (pattern_type)
  {
  case CHESSBOARD:
  case CIRCLES_GRID:
    for (int i = 0; i < board_size.height; i++)
    {
      for (int j = 0; j < board_size.width; j++)
      {
        corners.push_back(cv::Point3f(
            float(j*square_size),
            float(i*square_size),
            0)
        );
      }
    }
    break;

  case ASYMMETRIC_CIRCLES_GRID:
    for (int i = 0; i < board_size.height; i++)
    {
      for(int j = 0; j < board_size.width; j++)
      {
        corners.push_back(cv::Point3f(float((2*j + i % 2)*square_size),
                                  float(i*square_size), 0));
      }
    }
    break;

  default:
    CV_Error(cv::Error::StsBadArg, "Unknown pattern type\n");
  }
}

static bool calibrateSingleCamera(
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const cv::Size &image_size, const cv::Size &board_size,
    Pattern pattern_type, float square_size, float aspect_ratio, int flags,
    cv::Mat &camera_matrix, cv::Mat &dist_coeffs,
    std::vector<cv::Mat> &rvecs, std::vector<cv::Mat> &tvecs,
    std::vector<float> &reproj_errs,
    double &total_avg_err)
{
  camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  if (flags & cv::CALIB_FIX_ASPECT_RATIO)
  {
    camera_matrix.at<double>(0,0) = aspect_ratio;
  }

  dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

  std::vector<std::vector<cv::Point3f>> object_points(1);
  calculateChessboardCorners(board_size, square_size, pattern_type, object_points[0]);

  object_points.resize(image_points.size(), object_points[0]);

  cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, DBL_EPSILON);
  double rms = cv::calibrateCamera(
      object_points, image_points, image_size,
      camera_matrix, dist_coeffs,
      rvecs, tvecs,
      flags | cv::CALIB_FIX_K4 | cv::CALIB_FIX_K5,
      term_criteria
  );

  bool ok = cv::checkRange(camera_matrix) && cv::checkRange(dist_coeffs);

  total_avg_err = computeReprojectionErrors(
      object_points, image_points,
      rvecs, tvecs, camera_matrix, dist_coeffs,
      reproj_errs
  );

  return ok;
}


static void saveSingleCameraParams(
    const std::string &filename,
    const cv::Size &image_size, const cv::Size &board_size,
    float square_size, float aspect_ratio, int flags,
    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
    const std::vector<cv::Mat> &rvecs, const std::vector<cv::Mat> &tvecs,
    const std::vector<float> &reproj_errs,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    double total_avg_err)
{
  cv::FileStorage fs(filename, cv::FileStorage::WRITE);

  std::time_t tt;
  std::time(&tt);
  struct tm *t2 = std::localtime( &tt );
  char buf[1024];
  std::strftime( buf, sizeof(buf)-1, "%c", t2 );

  fs << "calibration_time" << buf;

  if (!rvecs.empty() || !reproj_errs.empty())
  {
    fs << "num_frames" << (int)std::max(rvecs.size(), reproj_errs.size());
  }
  fs << "image_width" << image_size.width;
  fs << "image_height" << image_size.height;
  fs << "board_width" << board_size.width;
  fs << "board_height" << board_size.height;
  fs << "square_size" << square_size;

  if (flags & cv::CALIB_FIX_ASPECT_RATIO)
  {
    fs << "aspect_ratio" << aspect_ratio;
  }

  if (flags != 0)
  {
    std::sprintf(buf, "flags: %s%s%s%s",
        flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
        flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspect_ratio" : "",
        flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
        flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    //cvWriteComment( *fs, buf, 0 );
  }

  fs << "flags" << flags;

  fs << "camera_matrix" << camera_matrix;
  fs << "distortion_coefficients" << dist_coeffs;

  fs << "avg_reprojection_error" << total_avg_err;
  if (!reproj_errs.empty())
  {
    fs << "per_view_reprojection_errors" << cv::Mat(reproj_errs);
  }

  if (!rvecs.empty() && !tvecs.empty())
  {
    CV_Assert(rvecs[0].type() == tvecs[0].type());
    cv::Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
    for (int i = 0; i < (int)rvecs.size(); i++)
    {
      cv::Mat r = bigmat(cv::Range(i, i+1), cv::Range(0,3));
      cv::Mat t = bigmat(cv::Range(i, i+1), cv::Range(3,6));

      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
      //*.t() is MatExpr (not Mat) so we can use assignment operator
      r = rvecs[i].t();
      t = tvecs[i].t();
    }
    //cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
    fs << "extrinsic_parameters" << bigmat;
  }

  if (!image_points.empty())
  {
    cv::Mat image_pt_mat((int)image_points.size(), (int)image_points[0].size(), CV_32FC2);
    for (int i = 0; i < (int)image_points.size(); i++)
    {
      cv::Mat r = image_pt_mat.row(i).reshape(2, image_pt_mat.cols);
      cv::Mat imgpti(image_points[i]);
      imgpti.copyTo(r);
    }
    fs << "image_points" << image_pt_mat;
  }
}

static bool calibrateAndSaveSingleCamera(
    const std::string &output_filename,
    const std::vector<std::vector<cv::Point2f>> &image_points,
    const cv::Size &image_size, const cv::Size &board_size, Pattern pattern_type, float square_size,
    float aspect_ratio, int flags,
    cv::Mat &camera_matrix, cv::Mat &dist_coeffs,
    bool write_extrinsics, bool write_points )
{
  std::vector<cv::Mat> rvecs, tvecs;
  std::vector<float> reproj_errs;
  double total_avg_err = 0;

  bool ok = calibrateSingleCamera(
      image_points, image_size, board_size, pattern_type, square_size,
      aspect_ratio, flags, camera_matrix, dist_coeffs,
      rvecs, tvecs, reproj_errs, total_avg_err
  );

  if (ok)
  {
    std::cout << "Calibration succeeded:" << std::endl;
    std::cout << "camera_matrix = " << camera_matrix << std::endl;
    std::cout << "dist_coeffs = " << dist_coeffs << std::endl;
    std::cout << "RMS error = " << total_avg_err << std::endl;
    std::cout << std::endl;

    if (!output_filename.empty())
    {
      saveSingleCameraParams(
          output_filename, image_size,
          board_size, square_size, aspect_ratio,
          flags, camera_matrix, dist_coeffs,
          write_extrinsics ? rvecs : std::vector<cv::Mat>(),
          write_extrinsics ? tvecs : std::vector<cv::Mat>(),
          write_extrinsics ? reproj_errs : std::vector<float>(),
          write_points ? image_points : std::vector<std::vector<cv::Point2f>>(),
          total_avg_err
      );
    }
  }

  return ok;
}

static bool calibrateStereoCamera(
    const cv::Size &image_size, const cv::Size &board_size,
    Pattern pattern_type, float square_size, float aspect_ratio, int flags,
    const std::vector<std::vector<cv::Point2f>> &image_points_left,
    const std::vector<std::vector<cv::Point2f>> &image_points_right,
    const cv::Mat &camera_matrix_left, const cv::Mat &dist_coeffs_left,
    const cv::Mat &camera_matrix_right, const cv::Mat &dist_coeffs_right,
    cv::Mat &rotation, cv::Mat &translation,
    cv::Mat &essential_matrix, cv::Mat &fundamental_matrix,
    double &total_avg_err)
{
  std::vector<std::vector<cv::Point3f>> object_points(1);
  calculateChessboardCorners(board_size, square_size, pattern_type, object_points[0]);

  object_points.resize(image_points_left.size(), object_points[0]);

  cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6);
  total_avg_err = cv::stereoCalibrate(
      object_points, image_points_left, image_points_right,
      camera_matrix_left, dist_coeffs_left,
      camera_matrix_right, dist_coeffs_right,
      image_size,
      rotation, translation,
      essential_matrix, fundamental_matrix,
      flags | CV_CALIB_FIX_INTRINSIC,
      term_criteria
  );

  bool ok = cv::checkRange(rotation) && cv::checkRange(translation)
    && cv::checkRange(essential_matrix) && cv::checkRange(fundamental_matrix);

  return ok;
}

static void saveStereoCameraParams(
    const std::string &output_filename,
    const cv::Size &image_size, const cv::Size &board_size,
    float square_size, float aspect_ratio, int flags,
    const cv::Mat &camera_matrix_left, const cv::Mat &dist_coeffs_left,
    const cv::Mat &camera_matrix_right, const cv::Mat &dist_coeffs_right,
    const std::vector<std::vector<cv::Point2f>> &image_points_left,
    const std::vector<std::vector<cv::Point2f>> &image_points_right,
    const cv::Mat &rotation, const cv::Mat &translation,
    const cv::Mat &essential_matrix, const cv::Mat &fundamental_matrix,
    double total_avg_err)
{
  cv::FileStorage fs(output_filename, cv::FileStorage::WRITE);

  std::time_t tt;
  std::time(&tt);
  struct tm *t2 = std::localtime(&tt);
  char buf[1024];
  std::strftime(buf, sizeof(buf)-1, "%c", t2);

  fs << "calibration_time" << buf;

  fs << "image_width" << image_size.width;
  fs << "image_height" << image_size.height;
  fs << "board_width" << board_size.width;
  fs << "board_height" << board_size.height;
  fs << "square_size" << square_size;

  if (flags & cv::CALIB_FIX_ASPECT_RATIO)
  {
    fs << "aspect_ratio" << aspect_ratio;
  }

  if (flags != 0)
  {
    std::sprintf(
        buf, "flags: %s%s%s%s",
        flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
        flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspect_ratio" : "",
        flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
        flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    //cv::cvWriteComment( *fs, buf, 0 );
  }

  fs << "flags" << flags;

  fs << "rotation" << rotation;
  fs << "translation" << translation;

  fs << "essential_matrix" << essential_matrix;
  fs << "fundamental_matrix" << fundamental_matrix;

  fs << "camera_matrix_left" << camera_matrix_left;
  fs << "distortion_coefficients_left" << dist_coeffs_left;

  fs << "camera_matrix_right" << camera_matrix_right;
  fs << "distortion_coefficients_right" << dist_coeffs_right;

  fs << "avg_reprojection_error" << total_avg_err;

  if (!image_points_left.empty())
  {
    cv::Mat image_pt_mat((int)image_points_left.size(), (int)image_points_left[0].size(), CV_32FC2);
    for (int i = 0; i < (int)image_points_left.size(); i++)
    {
      cv::Mat r = image_pt_mat.row(i).reshape(2, image_pt_mat.cols);
      cv::Mat imgpti(image_points_left[i]);
      imgpti.copyTo(r);
    }
    fs << "image_points_left" << image_pt_mat;
  }

  if (!image_points_right.empty())
  {
    cv::Mat image_pt_mat((int)image_points_right.size(), (int)image_points_right[0].size(), CV_32FC2);
    for (int i = 0; i < (int)image_points_right.size(); i++)
    {
      cv::Mat r = image_pt_mat.row(i).reshape(2, image_pt_mat.cols);
      cv::Mat imgpti(image_points_right[i]);
      imgpti.copyTo(r);
    }
    fs << "image_points_right" << image_pt_mat;
  }

  fs.release();
}

static bool calibrateAndSaveStereoCamera(
    const std::string &output_filename,
    const cv::Size &image_size, const cv::Size &board_size,
    Pattern pattern, float square_size, float aspect_ratio, int flags,
    const std::vector<std::vector<cv::Point2f>> &image_points_left,
    const std::vector<std::vector<cv::Point2f>> &image_points_right,
    const cv::Mat &camera_matrix_left, const cv::Mat &dist_coeffs_left,
    const cv::Mat &camera_matrix_right, const cv::Mat &dist_coeffs_right,
    bool write_extrinsics, bool write_points)
{
    double total_avg_err;

    cv::Mat rotation;
    cv::Mat translation;
    cv::Mat essential_matrix;
    cv::Mat fundamental_matrix;
    bool ok = calibrateStereoCamera(
        image_size, board_size, pattern, square_size, aspect_ratio, flags,
        image_points_left, image_points_right,
        camera_matrix_left, dist_coeffs_left,
        camera_matrix_right, dist_coeffs_right,
        rotation, translation,
        essential_matrix, fundamental_matrix,
        total_avg_err
    );

    if (ok)
    {
      std::cout << "Stereo calibration succeeded:" << std::endl;
      std::cout << "rotation = " << rotation << std::endl;
      std::cout << "translation = " << translation << std::endl;
      std::cout << "essential_matrix = " << essential_matrix << std::endl;
      std::cout << "fundamental_matrix = " << fundamental_matrix << std::endl;
      std::cout << "RMS error = " << total_avg_err << std::endl;
      std::cout << std::endl;

      saveStereoCameraParams(
          output_filename,
          image_size, board_size, square_size, aspect_ratio, flags,
          camera_matrix_left, dist_coeffs_left,
          camera_matrix_right, dist_coeffs_right,
          image_points_left, image_points_right,
          rotation, translation,
          essential_matrix, fundamental_matrix,
          total_avg_err
      );
    }

    return ok;
}

std::string showImagesAndWaitForCommand(
    const std::vector<std::pair<cv::Mat, std::string>> &images_and_names,
    const std::vector<std::pair<std::string, char>> &commands_and_keys,
    int key_wait_delay=50)
{
  CV_Assert(!images_and_names.empty());
  CV_Assert(!commands_and_keys.empty());
  while (true)
  {
    // Show images and commands
    for (const auto &image_and_name : images_and_names)
    {
      cv::Mat image_copy = image_and_name.first.clone();

      // Put command description onto image
      cv::Size text_size = cv::getTextSize("ABC", 1, 1, 1, nullptr);
      for (int i = 0; i < commands_and_keys.size(); ++i)
      {
        const auto &command_and_key = commands_and_keys[i];
        cv::Point text_origin(20, 20 + i * (10 + text_size.height));
        std::ostringstream msg_out;
        msg_out << "Key: " << (char)command_and_key.second << ", Command: " << command_and_key.first;
        cv::putText(image_copy, msg_out.str(), text_origin, 1, 1, cv::Scalar(0,0,255));
      }

      cv::imshow(image_and_name.second, image_copy);
    }

    // Check key input
    int key = 0xff & cv::waitKey(key_wait_delay);
    if (key== 27)
    {
      return std::string();
    }
    for (const auto &command_and_key : commands_and_keys)
    {
      if (key == command_and_key.second)
      {
        return command_and_key.first;
      }
    }
  }
}

int main(int argc, char **argv)
{
  try
  {
    TCLAP::CmdLine cmd("Stereo calibration tool", ' ', "0.1");
    TCLAP::ValueArg<int> device_arg("d", "device", "Device number to use", false, 0, "id", cmd);
    TCLAP::ValueArg<std::string> video_arg("v", "video", "Video device file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<std::string> svo_arg("", "svo", "SVO file to use", false, "", "filename", cmd);
    TCLAP::ValueArg<int> mode_arg("", "mode", "ZED Resolution mode", false, 2, "mode", cmd);
    TCLAP::ValueArg<double> fps_arg("", "fps", "Frame-rate to capture", false, 15, "Hz", cmd);
    TCLAP::ValueArg<std::string> zed_params_arg("", "zed-params", "ZED parameter file", false, "", "filename", cmd);
    TCLAP::ValueArg<int> board_width_arg("", "width", "Board width", true, 0, "number", cmd);
    TCLAP::ValueArg<int> board_height_arg("", "height", "Board height", true, 0, "number", cmd);
    TCLAP::ValueArg<std::string> pattern_arg("", "pattern", "Pattern (chessboard, circles, acircles", false, "chessboard", "pattern", cmd);
    TCLAP::ValueArg<double> scale_arg("s", "scale", "Square scale", false, 1.0, "au", cmd);
    TCLAP::ValueArg<int> num_frames_arg("n", "num_frames", "Number of frames", false, 10, "integer", cmd);
    TCLAP::ValueArg<double> aspect_ratio_arg("", "aspect-ratio", "Aspect ratio", false, 1.0, "double", cmd);
    TCLAP::ValueArg<std::string> output_filename_arg("o", "output-filename", "Output filename", false, "camera_calibration", "filename", cmd);
    TCLAP::ValueArg<std::string> frames_prefix_arg("", "frames-prefix", "Frames prefix", false, "frame", "string", cmd);
    TCLAP::SwitchArg live_capture_arg("", "capture", "Live capture", cmd, false);

    cmd.parse(argc, argv);

    bool undistort_image = false;
    bool live_capture = live_capture_arg.getValue();

    std::string output_filename_left = output_filename_arg.getValue() + "_left.yml";
    std::string output_filename_right = output_filename_arg.getValue() + "_right.yml";
    std::string output_filename_stereo = output_filename_arg.getValue() + "_stereo.yml";

    int flags = 0;
    bool showUndistorted;
    clock_t prev_timestamp = std::clock();
    Pattern pattern = CHESSBOARD;

    cv::Size board_size;
    board_size.width = board_width_arg.getValue();
    board_size.height = board_height_arg.getValue();
    std::string pattern_str = pattern_arg.getValue();
    if (pattern_str == "circles")
    {
      pattern = CIRCLES_GRID;
    }
    else if (pattern_str == "acircles")
    {
      pattern = ASYMMETRIC_CIRCLES_GRID;
    }
    else if (pattern_str == "chessboard")
    {
      pattern = CHESSBOARD;
    }
    else
    {
      std::cerr << "Invalid pattern type: " << pattern_str << std::endl;
    }
    double square_size = scale_arg.getValue();
    int num_frames = num_frames_arg.getValue();
    double aspect_ratio = aspect_ratio_arg.getValue();

    bool write_points = false;
    bool write_extrinsics = false;

//    bool write_points = write_points_arg.getValue();
//    bool write_extrinsics = write_extrinsics_arg.getValue();
    //    bool show_undistorted = show_undistorted_arg.getValue();
//    if (aspect_ratio_arg.isSet())
//    {
//      flags |= CALIB_FIX_ASPECT_RATIO;
//    }
//    if (no_tangent_dist_arg.isSet())
//    {
//      flags |= CALIB_ZERO_TANGENT_DIST;
//    }
//    if (fix_principal_point_arg.isSet())
//    {
//      flags |= CALIB_FIX_PRINCIPAL_POINT;
//    }

    if (square_size <= 0)
    {
      std::cerr << "Invalid board square width" << std::endl;
    }
    if (num_frames <= 3)
    {
      std::cerr << "Invalid number of images" << std::endl;
    }
    if (aspect_ratio <= 0)
    {
      std::cerr << "Invalid aspect ratio" << std::endl;
    }
    if (board_size.width <= 0)
    {
      std::cerr << "Invalid board width" << std::endl;
    }
    if (board_size.height <= 0)
    {
      std::cerr << "Invalid board height" << std::endl;
    }

    if (live_capture)
    {
      std::cout << liveCaptureHelp << std::endl;
    }

    video::VideoSourceZED video;

    if (live_capture)
    {
      if (zed_params_arg.isSet())
      {
        video.getInitParameters().load(zed_params_arg.getValue());
      }
  //    video.getInitParameters().disableSelfCalib = false;
      if (svo_arg.isSet())
      {
        video.open(svo_arg.getValue());
      }
      else
      {
        video.open(static_cast<sl::zed::ZEDResolution_mode>(mode_arg.getValue()));
      }
      video.getInitParameters().save("MyParam");

      if (video.setFPS(fps_arg.getValue()))
      {
        throw std::runtime_error("Unable to set ZED framerate");
      }
      std::cout << "ZED framerate: " << video.getFPS() << std::endl;
    }

    cv::namedWindow("Left", 1);
    cv::namedWindow("Right", 1);

    std::vector<std::string> image_list_left;
    std::vector<std::string> image_list_right;
    if (!live_capture)
    {
      image_list_left = readStringList(frames_prefix_arg.getValue() + "_left_list.txt");
      image_list_right = readStringList(frames_prefix_arg.getValue() + "_right_list.txt");
      std::cout << "Found " << image_list_left.size() << " left and "
          << image_list_right.size() << " right images" << std::endl;
    }

    cv::Mat camera_matrix_left;
    cv::Mat dist_coeffs_left;
    cv::Mat camera_matrix_right;
    cv::Mat dist_coeffs_right;

    std::vector<std::vector<cv::Point2f>> image_points_left;
    std::vector<std::vector<cv::Point2f>> image_points_right;

    std::ofstream list_file_left;
    std::ofstream list_file_right;
    if (live_capture)
    {
      list_file_left.open(frames_prefix_arg.getValue() + "_left_list.txt");
      list_file_right.open(frames_prefix_arg.getValue() + "_right_list.txt");
    }

    cv::Size image_size;
    int next_i = 0;
    int i = 0;
    cv::Mat view_left_orig;
    cv::Mat view_right_orig;
    cv::Mat view_left;
    cv::Mat view_right;
    while (i < num_frames)
    {
      if (live_capture)
      {
        video.grab();
        video.retrieveLeft(&view_left_orig);
        video.retrieveRight(&view_right_orig);
      }
      else
      {
        if (next_i == i)
        {
          std::cout << "Left image " << i << ": " << image_list_left[i] << std::endl;
          view_left_orig = cv::imread(image_list_left[i]);
          if (view_left_orig.data == nullptr)
          {
            throw std::runtime_error("Unable to read left image");
          }
          std::cout << "Right image " << i << ": " << image_list_right[i] << std::endl;
          view_right_orig = cv::imread(image_list_right[i]);
          if (view_right_orig.data == nullptr)
          {
            throw std::runtime_error("Unable to read right image");
          }
          ++next_i;
        }
      }
      view_left_orig.copyTo(view_left);
      view_right_orig.copyTo(view_right);

      image_size = view_left.size();
      CV_Assert(image_size == view_right.size());

      std::vector<cv::Point2f> pointbuf_left;
      std::vector<cv::Point2f> pointbuf_right;

      cv::Mat view_left_gray;
      cv::Mat view_right_gray;
      cv::cvtColor(view_left, view_left_gray, cv::COLOR_BGR2GRAY);
      cv::cvtColor(view_right, view_right_gray, cv::COLOR_BGR2GRAY);

      bool found_left = false;
      bool found_right = false;
      switch (pattern)
      {
      case CHESSBOARD:
        found_left = cv::findChessboardCorners(view_left, board_size, pointbuf_left,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        found_right= cv::findChessboardCorners(view_right, board_size, pointbuf_right,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        break;
      case CIRCLES_GRID:
        found_left = cv::findCirclesGrid(view_left, board_size, pointbuf_left);
        found_right = cv::findCirclesGrid(view_right, board_size, pointbuf_right);
        break;
      case ASYMMETRIC_CIRCLES_GRID:
        found_left = cv::findCirclesGrid(view_left, board_size, pointbuf_left, cv::CALIB_CB_ASYMMETRIC_GRID);
        found_right = cv::findCirclesGrid(view_right, board_size, pointbuf_right, cv::CALIB_CB_ASYMMETRIC_GRID);
        break;
      default:
        return std::fprintf( stderr, "Unknown pattern type\n" ), -1;
      }

     // improve the found corners' coordinate accuracy
      if (pattern == CHESSBOARD)
      {
        if (found_left)
        {
          cv::cornerSubPix(view_left_gray, pointbuf_left, cv::Size(11,11),
              cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1));
        }
        if (found_right)
        {
          cv::cornerSubPix(view_right_gray, pointbuf_right, cv::Size(11,11),
              cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1));
        }
      }

      if (found_left)
      {
        cv::drawChessboardCorners(view_left, board_size, cv::Mat(pointbuf_left), found_left);
      }
      if (found_right)
      {
        cv::drawChessboardCorners(view_right, board_size, cv::Mat(pointbuf_right), found_right);
      }

      std::string msg = "-------------------------------------------------";
      int baseLine = 0;
      cv::Size textSize = cv::getTextSize(msg, 1, 1, 1, &baseLine);
      cv::Point text_origin(view_left.cols - 2*textSize.width - 10, view_left.rows - 2*baseLine - 10);

      if (undistort_image)
      {
          msg = cv::format("%d/%d Undist", i, num_frames);
      }
      else
      {
          msg = cv::format("%d/%d", i, num_frames);
      }

      cv::putText(view_left, msg, text_origin, 1, 1, cv::Scalar(0,0,255));

      if (undistort_image)
      {
        cv::Mat temp = view_left.clone();
        cv::undistort(temp, view_left, camera_matrix_left, dist_coeffs_left);
        temp = view_right.clone();
        cv::undistort(temp, view_right, camera_matrix_right, dist_coeffs_right);
      }

      cv::imshow("Left", view_left);
      cv::imshow("Right", view_right);
      int key = 0xff & cv::waitKey(live_capture ? 10 : 500);

      if ((key & 255) == 27 || key == 'q')
      {
        break;
      }

      if (key == 'u')
      {
        undistort_image = !undistort_image;
      }

      if (live_capture)
      {
        if (key == 'r')
        {
          std::cout << "Clearing all frames" << std::endl;
          image_points_left.clear();
          image_points_right.clear();
          list_file_left.close();
          list_file_right.close();
          list_file_left.open(frames_prefix_arg.getValue() + "_left_list.txt");
          list_file_right.open(frames_prefix_arg.getValue() + "_right_list.txt");
          i = 0;
        }

        if (key == 'c')
        {
          std::clock_t current_timestamp = std::clock();
          std::clock_t d_stamp = current_timestamp - prev_timestamp;
          if (d_stamp > 0.1 * CLOCKS_PER_SEC && found_left && found_right)
          {
            std::vector<std::pair<cv::Mat, std::string>> images_and_names = {{view_left, "Left"}, {view_right, "Right"}};
            std::vector<std::pair<std::string, char>> commands_and_keys = {{"keep", 'k'}, {"discard", 'd'}};
            std::string command = showImagesAndWaitForCommand(images_and_names, commands_and_keys);
            if (command == "keep")
            {
              prev_timestamp = current_timestamp;
              std::cout << "Keeping frame " << i << std::endl;
              image_points_left.push_back(pointbuf_left);
              image_points_right.push_back(pointbuf_right);

              std::ostringstream out;
              out << frames_prefix_arg.getValue() << "_left_" << i << ".png";
              cv::imwrite(out.str(), view_left_orig);
              list_file_left << out.str() << std::endl;

              out.str("");
              out.clear();
              out << frames_prefix_arg.getValue() << "_right_" << i << ".png";
              cv::imwrite(out.str(), view_right_orig);
              list_file_right << out.str() << std::endl;
              ++i;
            }
            else if (command.empty())
            {
              break;
            }
          }
        }
      }
      else
      {
        if (key == 'n')
        {
          if (!found_left || !found_right)
          {
            throw std::runtime_error("Could not find corners in recorded images");
          }
          image_points_left.push_back(pointbuf_left);
          image_points_right.push_back(pointbuf_right);
          ++i;
        }
      }
    }
    list_file_left.close();
    list_file_right.close();

    if (i >= num_frames)
    {
      std::cout << "Calibrating left camera ..." << std::endl;
      bool success_left = calibrateAndSaveSingleCamera(
          output_filename_left, image_points_left, image_size,
          board_size, pattern, square_size, aspect_ratio,
          flags, camera_matrix_left, dist_coeffs_left,
          write_extrinsics, write_points
      );
      std::cout << "Calibrating right camera ..." << std::endl;
      bool success_right = calibrateAndSaveSingleCamera(
          output_filename_right, image_points_right, image_size,
          board_size, pattern, square_size, aspect_ratio,
          flags, camera_matrix_right, dist_coeffs_right,
          write_extrinsics, write_points
      );
      if (success_left && success_right)
      {
        std::cout << "Calibrating stereo camera ..." << std::endl;
        bool success_stereo = calibrateAndSaveStereoCamera(
            output_filename_stereo,
            image_size, board_size, pattern, square_size, aspect_ratio, flags,
            image_points_left, image_points_right,
            camera_matrix_left, dist_coeffs_left,
            camera_matrix_right, dist_coeffs_right,
            write_extrinsics, write_points
        );
      }
    }
  }
  catch (TCLAP::ArgException &err)
  {
    std::cerr << "Command line error: " << err.error() << " for arg " << err.argId() << std::endl;
  }

  return 0;
}
