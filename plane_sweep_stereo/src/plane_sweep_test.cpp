// This file is part of PlaneSweepLib (PSL)

// Copyright 2016 Christian Haene (ETH Zuerich)

// PSL is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// PSL is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with PSL.  If not, see <http://www.gnu.org/licenses/>.

#include <boost/program_options.hpp>
#include <psl_base/exception.h>
#include <fstream>
#include <map>
#include <Eigen/Dense>
#include <psl_base/cameraMatrix.h>
#include <psl_stereo/cudaPlaneSweep.h>
#include <opencv2/highgui/highgui.hpp>
#include <boost/filesystem.hpp>
#include <iostream>
#include <ait/stereo/stereo_calibration.h>
#include <ait/utilities.h>

void makeOutputFolder(std::string folderName)
{
    if (!boost::filesystem::exists(folderName))
    {
        if (!boost::filesystem::create_directory(folderName))
        {
            std::stringstream errorMsg;
            errorMsg << "Could not create output directory: " << folderName;
            PSL_THROW_EXCEPTION(errorMsg.str().c_str());
        }
    }
}

int main(int argc, char* argv[])
{
  namespace po = boost::program_options;
  namespace ast = ait::stereo;

  std::string data_folder;
  std::string calib_file;
  std::string img_left_file;
  std::string img_right_file;

  try
  {
    po::options_description desc("Allowed options");
    desc.add_options()
            ("help", "Produce help message")
            ("data-folder", po::value<std::string>(&data_folder)->default_value("DataPinholeCamera/niederdorf2"), "One of the data folders for pinhole planesweep provided with the plane sweep code.")
            ("calib-file", po::value<std::string>(&calib_file)->default_value("camera_calibration_stereo.yml"), "Stereo calibration file.")
            ("left-img", po::value<std::string>(&img_left_file)->required(), "Left frame.")
            ("right-img", po::value<std::string>(&img_right_file)->required(), "Right frame.")
            ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).run(), vm);
    if (vm.count("help"))
    {
        std::cout << desc << std::endl;
        return 1;
    }

    po::notify(vm);

    // Try to read stereo camera calibration
    ast::StereoCameraCalibration calib = ast::StereoCameraCalibration::readStereoCalibration(calib_file);
    Eigen::Matrix3d K_left = calib.left.getCameraMatrixEigen();
    Eigen::Matrix3d R_left = Eigen::Matrix3d::Identity();
    Eigen::Vector3d T_left = Eigen::Vector3d::Zero();
    Eigen::Matrix3d K_right = calib.right.getCameraMatrixEigen();
    Eigen::Matrix3d R_right = calib.getRotationEigen();
    Eigen::Vector3d T_right = calib.getTranslationEigen();

    // Create PlaneSweepLib cameras
    PSL::CameraMatrix<double> camera_left;
    PSL::CameraMatrix<double> camera_right;
    camera_left.setKRT(K_left, R_left, T_left);
    camera_right.setKRT(K_right, R_right, T_right);

    std::cout << "Left center:" << std::endl << camera_left.getC() << std::endl;
    std::cout << "Right center:" << std::endl << camera_right.getC() << std::endl;

    // Load images
    cv::Mat img_left = cv::imread(img_left_file, cv::IMREAD_COLOR);
    if (img_left.data == nullptr)
    {
      throw std::runtime_error("Unable to read left image");
    }
    cv::Mat img_right = cv::imread(img_right_file, cv::IMREAD_COLOR);
    if (img_left.data == nullptr)
    {
      throw std::runtime_error("Unable to read right image");
    }

    // The reconstructions are not metric. In order to have an idea about the scale
    // everything is defined with respect to the average distance between the cameras.
    Eigen::Vector3d distance = camera_left.getC() - camera_right.getC();
    double avg_distance = distance.norm();

    //    float minZ = 0.5f * 1000;
    //    float maxZ = 5.0f * 1000;
    float minZ = (float) (2.5f*avg_distance);
    float maxZ = (float) (100.0f*avg_distance);
    std::cout << "minZ=" << minZ << ", maxZ=" << maxZ << std::endl;
    minZ = 0.4f;
    maxZ = 5.0f;
    std::cout << "minZ=" << minZ << ", maxZ=" << maxZ << std::endl;

    double scale = 1.0;
    int window_size = 15;
    int num_planes = 128;

    makeOutputFolder("test_results");

    cv::imshow("Color Image", img_left);

    // First tests compute a depth map for the middle image of the first row
    {
        makeOutputFolder("test_results/colorSAD");

        PSL::CudaPlaneSweep cPS;
        cPS.setScale(scale); // Scale the images down to 0.25 times the original side length
        cPS.setZRange(minZ, maxZ);
        cPS.setMatchWindowSize(window_size, window_size);
        cPS.setNumPlanes(num_planes);
        cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_NONE);
        cPS.setPlaneGenerationMode(PSL::PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
        cPS.setMatchingCosts(PSL::PLANE_SWEEP_SAD);
        cPS.setSubPixelInterpolationMode(PSL::PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
        cPS.enableOutputBestDepth();
        cPS.enableColorMatching();
        cPS.enableOutputBestCosts(false);
        cPS.enableOuputUniquenessRatio(false);
        cPS.enableOutputCostVolume(false);
        cPS.enableSubPixel();

        // now we upload the images
        int refId = 0;
        ait::ProfilingTimer timer2;
        int id_left = cPS.addImage(img_left, camera_left);
        int id_right = cPS.addImage(img_right, camera_right);
        timer2.stopAndPrintTiming("Image upload to GPU");

        {
            ait::ProfilingTimer timer;
            cPS.process(refId);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refId);
            timer.stopAndPrintTiming("Plane sweep stereo with color SAD");

            makeOutputFolder("test_results/colorSAD/NoOcclusionHandling/");
            cv::imwrite("test_results/colorSAD/NoOcclusionHandling/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("test_results/colorSAD/NoOcclusionHandling/invDepthCol.png", minZ, maxZ);

//            cv::imshow("Reference Image", refImage);
            dM.displayInvDepthColored(minZ, maxZ, 100, "Color SAD");
        }
    }

    // First tests compute a depth map for the middle image of the first row
    {
        makeOutputFolder("test_results/grayscaleSAD");
        makeOutputFolder("test_results/grayscaleZNCC");

        PSL::CudaPlaneSweep cPS;
        cPS.setScale(scale); // Scale the images down to 0.25 times the original side length
        cPS.setZRange(minZ, maxZ);
        cPS.setMatchWindowSize(window_size, window_size);
        cPS.setNumPlanes(num_planes);
        cPS.setOcclusionMode(PSL::PLANE_SWEEP_OCCLUSION_NONE);
        cPS.setPlaneGenerationMode(PSL::PLANE_SWEEP_PLANEMODE_UNIFORM_DISPARITY);
        cPS.setSubPixelInterpolationMode(PSL::PLANE_SWEEP_SUB_PIXEL_INTERP_INVERSE);
        cPS.enableOutputBestDepth();
        cPS.enableColorMatching(false);
        cPS.enableOutputBestCosts(false);
        cPS.enableOuputUniquenessRatio(false);
        cPS.enableOutputCostVolume(false);
        cPS.enableSubPixel();

        // now we upload the images
        int refId = 0;
        int id_left = cPS.addImage(img_left, camera_left);
        int id_right = cPS.addImage(img_right, camera_right);

        {
            ait::ProfilingTimer timer;
            cPS.setMatchingCosts(PSL::PLANE_SWEEP_SAD);
            cPS.process(refId);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refId);
            timer.stopAndPrintTiming("Plane sweep stereo with grayscale SAD");

            makeOutputFolder("test_results/grayscaleSAD/NoOcclusionHandling/");
            cv::imwrite("test_results/grayscaleSAD/NoOcclusionHandling/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("test_results/grayscaleSAD/NoOcclusionHandling/invDepthCol.png", minZ, maxZ);

//            cv::imshow("Reference Image", refImage);
            dM.displayInvDepthColored(minZ, maxZ, 100, "Grayscale SAD");
        }

        {
            ait::ProfilingTimer timer;
            cPS.setMatchingCosts(PSL::PLANE_SWEEP_ZNCC);
            cPS.process(refId);
            PSL::DepthMap<float, double> dM;
            dM = cPS.getBestDepth();
            cv::Mat refImage = cPS.downloadImage(refId);
            timer.stopAndPrintTiming("Plane sweep stereo with grayscale ZNCC");

            makeOutputFolder("test_results/grayscaleZNCC/NoOcclusionHandling/");
            cv::imwrite("test_results/grayscaleZNCC/NoOcclusionHandling/refImg.png",refImage);
            dM.saveInvDepthAsColorImage("test_results/grayscaleZNCC/NoOcclusionHandling/invDepthCol.png", minZ, maxZ);

//            cv::imshow("Reference Image", refImage);
            dM.displayInvDepthColored(minZ, maxZ, 100, "Grayscale ZNCC");
        }

        cv::waitKey();
    }

  }
  catch (const po::required_option &err)
  {
    std::cerr << "Error parsing command line: Required option '" << err.get_option_name() << "' is missing" << std::endl;
  }
  catch (const po::error &err)
  {
    std::cerr << "Error parsing command line: " << err.what() << std::endl;
  }

  return 0;
}
