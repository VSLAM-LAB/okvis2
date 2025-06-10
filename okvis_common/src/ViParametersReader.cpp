/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *  Copyright (c) 2020, Smart Robotics Lab / Imperial College London
 *  Copyright (c) 2024, Smart Robotics Lab / Technical University of Munich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab, ETH Zurich, Smart Robotics Lab,
 *     Imperial College London, Technical University of Munich, nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************************/

/**
 * @file ViParametersReader.cpp
 * @brief Source file for the VioParametersReader class.
 * @author Stefan Leutenegger
 * @author Andreas Forster
 */

#include <algorithm>

#include <glog/logging.h>

#include <okvis/cameras/PinholeCamera.hpp>
#include <okvis/cameras/EquidistantDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion.hpp>
#include <okvis/cameras/RadialTangentialDistortion8.hpp>

#include <opencv2/core/core.hpp>

#include <okvis/ViParametersReader.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// The default constructor.
ViParametersReader::ViParametersReader()
    : readConfigFile_(false) {
}

// The constructor. This calls readConfigFile(). 
ViParametersReader::ViParametersReader(const std::string& calibration_yaml, const std::string& settings_yaml) {
  // reads
  readConfigFile(calibration_yaml, settings_yaml);
}

// Read and parse a config file.
void ViParametersReader::readConfigFile(const std::string& calibration_yaml, const std::string& settings_yaml) {

  // reads
  cv::FileStorage fCalib(calibration_yaml, cv::FileStorage::READ);
  cv::FileStorage file(settings_yaml, cv::FileStorage::READ);

  OKVIS_ASSERT_TRUE(Exception, file.isOpened(),
                    "Could not open config file: " << settings_yaml)

  OKVIS_ASSERT_TRUE(Exception, fCalib.isOpened(),
                    "Could not open calib file: " << calibration_yaml)   

  LOG(INFO) << "Opened configuration file: " << settings_yaml;
  LOG(INFO) << "Opened configuration file: " << calibration_yaml;

  // camera calibration
  std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> calibrations;
  if(!getCameraCalibration(calibrations, fCalib)) {
    LOG(FATAL) << "Did not find any calibration!";
  }

  size_t camIdx = 0;
  for (size_t i = 0; i < calibrations.size(); ++i) {

    std::shared_ptr<const kinematics::Transformation> T_SC_okvis_ptr(
          new kinematics::Transformation(calibrations[i].T_SC.r(),
                                                calibrations[i].T_SC.q().normalized()));
    if (strcmp(calibrations[i].distortionType.c_str(), "equidistant") == 0) {
      std::shared_ptr<cameras::PinholeCamera<cameras::EquidistantDistortion>> cam;
      cam.reset(new cameras::PinholeCamera<
                  cameras::EquidistantDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::EquidistantDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::Equidistant, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Equidistant pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob") == 0) {
      std::shared_ptr<cameras::PinholeCamera<cameras::RadialTangentialDistortion>> cam;
      cam.reset(new cameras::PinholeCamera<
                  cameras::RadialTangentialDistortion>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::RadialTangentialDistortion(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::RadialTangential, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else if (strcmp(calibrations[i].distortionType.c_str(), "radialtangential8") == 0
               || strcmp(calibrations[i].distortionType.c_str(), "plumb_bob8") == 0) {
      std::shared_ptr<cameras::PinholeCamera<cameras::RadialTangentialDistortion8>> cam;
      cam.reset(new cameras::PinholeCamera<cameras::RadialTangentialDistortion8>(
                  calibrations[i].imageDimension[0],
                  calibrations[i].imageDimension[1],
                  calibrations[i].focalLength[0],
                  calibrations[i].focalLength[1],
                  calibrations[i].principalPoint[0],
                  calibrations[i].principalPoint[1],
                  cameras::RadialTangentialDistortion8(
                    calibrations[i].distortionCoefficients[0],
                    calibrations[i].distortionCoefficients[1],
                    calibrations[i].distortionCoefficients[2],
                    calibrations[i].distortionCoefficients[3],
                    calibrations[i].distortionCoefficients[4],
                    calibrations[i].distortionCoefficients[5],
                    calibrations[i].distortionCoefficients[6],
                    calibrations[i].distortionCoefficients[7])/*, id ?*/));
      cam->initialiseUndistortMaps(); // set up undistorters
      cam->initialiseCameraAwarenessMaps();
      viParameters_.nCameraSystem.addCamera(
          T_SC_okvis_ptr,
          std::static_pointer_cast<const cameras::CameraBase>(cam),
          cameras::NCameraSystem::RadialTangential8, true,
          calibrations[i].cameraType);
      std::stringstream s;
      s << calibrations[i].T_SC.T();
      LOG(INFO) << "Radial tangential 8 pinhole camera " << camIdx
                << " with T_SC=\n" << s.str();
    } else {
      LOG(ERROR) << "unrecognized distortion type " << calibrations[i].distortionType;
    }
    ++camIdx;
  }

  //camera parameters.
  parseEntry(file["camera_parameters"], "timestamp_tolerance",
             viParameters_.camera.timestamp_tolerance);
  cv::FileNode T = file["camera_parameters"]["sync_cameras"];
  OKVIS_ASSERT_TRUE(
    Exception, T.isSeq(),
    "missing real array parameter " << "camera_parameters" << ": " << "sync_cameras")
  for(auto iter = T.begin(); iter != T.end(); ++iter) {
    viParameters_.camera.sync_cameras.insert(int(*iter));
  }
  parseEntry(file["camera_parameters"], "image_delay",
             viParameters_.camera.image_delay);
  parseEntry(file["camera_parameters"]["online_calibration"], "do_extrinsics",
             viParameters_.camera.online_calibration.do_extrinsics);
  parseEntry(file["camera_parameters"]["online_calibration"], "do_extrinsics_final_ba",
             viParameters_.camera.online_calibration.do_extrinsics_final_ba);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_r",
             viParameters_.camera.online_calibration.sigma_r);
  parseEntry(file["camera_parameters"]["online_calibration"], "sigma_alpha",
             viParameters_.camera.online_calibration.sigma_alpha);

  //IMU parameters.
  parseEntry(file["imu_parameters"], "use",
             viParameters_.imu.use);
  Eigen::Matrix4d T_BS;
  parseEntry(file["imu_parameters"], "T_BS", T_BS);
  viParameters_.imu.T_BS = kinematics::Transformation(T_BS);
  parseEntry(file["imu_parameters"], "a_max",
             viParameters_.imu.a_max);
  parseEntry(file["imu_parameters"], "g_max",
             viParameters_.imu.g_max);

  //parseEntry(fCalib["IMU"], "NoiseGyro",
  //           viParameters_.imu.sigma_g_c);
  viParameters_.imu.sigma_g_c = fCalib["IMU.NoiseGyro"];

  parseEntry(file["imu_parameters"], "sigma_bg",
             viParameters_.imu.sigma_bg);

  //parseEntry(file["imu_parameters"], "sigma_a_c",
  //           viParameters_.imu.sigma_a_c);
  viParameters_.imu.sigma_a_c = fCalib["IMU.NoiseAcc"];

  parseEntry(file["imu_parameters"], "sigma_ba",
             viParameters_.imu.sigma_ba);

  //parseEntry(file["imu_parameters"], "sigma_gw_c",
  //           viParameters_.imu.sigma_gw_c);
  viParameters_.imu.sigma_gw_c = fCalib["IMU.GyroWalk"];
       
  //parseEntry(file["imu_parameters"], "sigma_aw_c",
  //           viParameters_.imu.sigma_aw_c);
  viParameters_.imu.sigma_aw_c = fCalib["IMU.AccWalk"];

  parseEntry(file["imu_parameters"], "a0",
             viParameters_.imu.a0);
  parseEntry(file["imu_parameters"], "g0",
             viParameters_.imu.g0);
  parseEntry(file["imu_parameters"], "g",
             viParameters_.imu.g);

  // Parameters for detection etc.
  parseEntry(file["frontend_parameters"], "detection_threshold",
             viParameters_.frontend.detection_threshold);
  parseEntry(file["frontend_parameters"], "absolute_threshold",
             viParameters_.frontend.absolute_threshold);
  parseEntry(file["frontend_parameters"], "matching_threshold",
             viParameters_.frontend.matching_threshold);
  parseEntry(file["frontend_parameters"], "octaves",
             viParameters_.frontend.octaves);
  parseEntry(file["frontend_parameters"], "max_num_keypoints",
             viParameters_.frontend.max_num_keypoints);
  parseEntry(file["frontend_parameters"], "keyframe_overlap",
             viParameters_.frontend.keyframe_overlap);
  parseEntry(file["frontend_parameters"], "use_cnn",
             viParameters_.frontend.use_cnn);
  parseEntry(file["frontend_parameters"], "parallelise_detection",
             viParameters_.frontend.parallelise_detection);
  parseEntry(file["frontend_parameters"], "num_matching_threads",
             viParameters_.frontend.num_matching_threads);

  // Parameters regarding the estimator.
  parseEntry(file["estimator_parameters"], "num_keyframes",
             viParameters_.estimator.num_keyframes);
  parseEntry(file["estimator_parameters"], "num_loop_closure_frames",
             viParameters_.estimator.num_loop_closure_frames);
  parseEntry(file["estimator_parameters"], "num_imu_frames",
             viParameters_.estimator.num_imu_frames);
  parseEntry(file["estimator_parameters"], "do_loop_closures",
             viParameters_.estimator.do_loop_closures);
  parseEntry(file["estimator_parameters"], "do_final_ba",
             viParameters_.estimator.do_final_ba);
  parseEntry(file["estimator_parameters"], "enforce_realtime",
             viParameters_.estimator.enforce_realtime);
  parseEntry(file["estimator_parameters"], "realtime_min_iterations",
             viParameters_.estimator.realtime_min_iterations);
  parseEntry(file["estimator_parameters"], "realtime_max_iterations",
             viParameters_.estimator.realtime_max_iterations);
  parseEntry(file["estimator_parameters"], "realtime_time_limit",
             viParameters_.estimator.realtime_time_limit);
  parseEntry(file["estimator_parameters"], "realtime_num_threads",
             viParameters_.estimator.realtime_num_threads);
  parseEntry(file["estimator_parameters"], "full_graph_iterations",
             viParameters_.estimator.full_graph_iterations);
  parseEntry(file["estimator_parameters"], "full_graph_num_threads",
             viParameters_.estimator.full_graph_num_threads);

  // Some options for how and what to output.
  parseEntry(file["output_parameters"], "display_matches",
             viParameters_.output.display_matches);
  parseEntry(file["output_parameters"], "display_overhead",
             viParameters_.output.display_overhead);

  // done!
  readConfigFile_ = true;
}

void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name, int& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isInt(),
                    "missing integer parameter " << file.name() << ": " << name)
  file[name] >> readValue;
}

void ViParametersReader::parseEntry(const cv::FileNode &file,
                                    std::string name, double& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isReal(),
                    "missing real parameter " << file.name() << ": " << name)
  file[name] >> readValue;
}

// Parses booleans from a cv::FileNode. OpenCV sadly has no implementation like this.
void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name, bool& readValue) {
  OKVIS_ASSERT_TRUE(Exception, file[name].isInt() || file[name].isString(),
                    "missing boolean parameter " << file.name() << ": " << name)
  if (file[name].isInt()) {
    readValue = int(file[name]) != 0;
    return;
  }
  if (file[name].isString()) {
    std::string str = std::string(file[name]);
    // cut out first word. str currently contains everything including comments
    str = str.substr(0,str.find(" "));
    // transform it to all lowercase
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    /* from yaml.org/type/bool.html:
     * Booleans are formatted as English words
     * (“true”/“false”, “yes”/“no” or “on”/“off”)
     * for readability and may be abbreviated as
     * a single character “y”/“n” or “Y”/“N”. */
    if (str.compare("false")  == 0
        || str.compare("no")  == 0
        || str.compare("n")   == 0
        || str.compare("off") == 0) {
      readValue = false;
      return;
    }
    if (str.compare("true")   == 0
        || str.compare("yes") == 0
        || str.compare("y")   == 0
        || str.compare("on")  == 0) {
      readValue = true;
      return;
    }
    OKVIS_THROW(Exception, "Boolean with uninterpretable value " << str)
  }
  return;
}

void ViParametersReader::parseEntry(const cv::FileNode &file, std::string name,
                                    Eigen::Matrix4d& readValue) {
  cv::FileNode T = file[name];
  OKVIS_ASSERT_TRUE(Exception, T.isSeq(),
                    "missing real array parameter " << file.name() << ": " << name)
  readValue << T[0], T[1], T[2], T[3],
               T[4], T[5], T[6], T[7],
               T[8], T[9], T[10], T[11],
               T[12], T[13], T[14], T[15];
}

void ViParametersReader::parseEntry(const cv::FileNode& file, std::string name,
                                    Eigen::Vector3d& readValue) {
  cv::FileNode T = file[name];
  OKVIS_ASSERT_TRUE(Exception, T.isSeq(),
                    "missing real array parameter " << file.name() << ": " << name)
  readValue << T[0], T[1], T[2];
}

bool ViParametersReader::getCameraCalibration(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
    cv::FileStorage& fCalib) {

  bool success = getCalibrationViaConfig(calibrations, fCalib);
  return success;
}

// Get the camera calibration via the configuration file.
bool ViParametersReader::getCalibrationViaConfig(
    std::vector<CameraCalibration,Eigen::aligned_allocator<CameraCalibration>> & calibrations,
    cv::FileStorage& fCalib) const {

  calibrations.clear();
  CameraCalibration calib;

  calib.imageDimension << fCalib["Camera.w"], fCalib["Camera.h"];

  std::string distortionType = (std::string)fCalib["Camera.model"];
  if (distortionType == "PINHOLE") {
    calib.distortionType = "radialtangential";
    calib.focalLength << fCalib["Camera.fx"], fCalib["Camera.fy"];
    calib.principalPoint << fCalib["Camera.cx"], fCalib["Camera.cy"];
    calib.distortionCoefficients.resize(4);
    calib.distortionCoefficients[0] = 0.0;
    calib.distortionCoefficients[1] = 0.0;
    calib.distortionCoefficients[2] = 0.0;
    calib.distortionCoefficients[3] = 0.0;
  }
  else if (distortionType == "OPENCV") {
    calib.distortionType = "radialtangential";
    calib.focalLength << fCalib["Camera.fx"], fCalib["Camera.fy"];
    calib.principalPoint << fCalib["Camera.cx"], fCalib["Camera.cy"];
    calib.distortionCoefficients.resize(4);
    calib.distortionCoefficients[0] = fCalib["Camera.k1"];
    calib.distortionCoefficients[1] = fCalib["Camera.k2"];
    calib.distortionCoefficients[2] = fCalib["Camera.p1"];
    calib.distortionCoefficients[3] = fCalib["Camera.p2"];
    //calib.distortionCoefficients[4] = fCalib["Camera.k3"];
  }

  cv::Mat cvTbc = fCalib["IMU.T_b_c1"].mat();
  Eigen::Matrix4f T_SC{};
  T_SC << cvTbc.at<float>(0,0), cvTbc.at<float>(0,1), cvTbc.at<float>(0,2), cvTbc.at<float>(0,3),
          cvTbc.at<float>(1,0), cvTbc.at<float>(1,1), cvTbc.at<float>(1,2), cvTbc.at<float>(1,3),
          cvTbc.at<float>(2,0), cvTbc.at<float>(2,1), cvTbc.at<float>(2,2), cvTbc.at<float>(2,3),
          cvTbc.at<float>(3,0), cvTbc.at<float>(3,1), cvTbc.at<float>(3,2), cvTbc.at<float>(3,3);
  calib.T_SC = kinematics::Transformation(T_SC.cast<double>());

  calib.cameraType.isColour = false;

  calib.cameraType.depthType.isDepthCamera = false;
  calib.cameraType.depthType.createDepth = false;
  calib.cameraType.depthType.createVirtual = false;
  calib.cameraType.isUsed = true;
  calib.cameraType.depthType.sigmaPixels = 0.0;
  
  calibrations.push_back(calib);

  return true;

  }

  //std::terminate();
  // first check if calibration is available in config file
  // if (cameraNode.isSeq()
  //    && cameraNode.size() > 0) {
  //   size_t camIdx = 0;
  //   for (cv::FileNodeIterator it = cameraNode.begin();
  //       it != cameraNode.end(); ++it) {
  //     if ((*it).isMap()
  //         && (*it)["T_SC"].isSeq()
  //         && (*it)["image_dimension"].isSeq()
  //         && (*it)["image_dimension"].size() == 2
  //         && (*it)["distortion_coefficients"].isSeq()
  //         && (*it)["distortion_coefficients"].size() >= 4
  //         && (*it)["distortion_type"].isString()
  //         && (*it)["focal_length"].isSeq()
  //         && (*it)["focal_length"].size() == 2
  //         && (*it)["principal_point"].isSeq()
  //         && (*it)["principal_point"].size() == 2) {
  //       LOG(INFO) << "Found calibration in configuration file for camera " << camIdx;
  //       gotCalibration = true;
  //     } else {
  //       LOG(WARNING) << "Found incomplete calibration in configuration file for camera " << camIdx
  //                    << ". Will not use the calibration from the configuration file.";
  //       return false;
  //     }
  //     ++camIdx;
  //   }
  // }
  // else
  //   LOG(INFO) << "Did not find a calibration in the configuration file.";
  // gotCalibration = true;   
  // if (gotCalibration) {
  //   for (cv::FileNodeIterator it = cameraNode.begin();
  //       it != cameraNode.end(); ++it) {

  //     CameraCalibration calib;

  //     cv::FileNode T_SC_node = (*it)["T_SC"];
  //     cv::FileNode imageDimensionNode = (*it)["image_dimension"];
  //     cv::FileNode distortionCoefficientNode = (*it)["distortion_coefficients"];
  //     cv::FileNode focalLengthNode = (*it)["focal_length"];
  //     cv::FileNode principalPointNode = (*it)["principal_point"];

  //     // extrinsics
  //     Eigen::Matrix4d T_SC;
  //     T_SC << T_SC_node[0], T_SC_node[1], T_SC_node[2], T_SC_node[3],
  //             T_SC_node[4], T_SC_node[5], T_SC_node[6], T_SC_node[7],
  //             T_SC_node[8], T_SC_node[9], T_SC_node[10], T_SC_node[11],
  //             T_SC_node[12], T_SC_node[13], T_SC_node[14], T_SC_node[15];
  //     calib.T_SC = kinematics::Transformation(T_SC);

  //     calib.imageDimension << imageDimensionNode[0], imageDimensionNode[1];
  //     calib.distortionCoefficients.resize(int(distortionCoefficientNode.size()));
  //     for(int i=0; i<int(distortionCoefficientNode.size()); ++i) {
  //       calib.distortionCoefficients[i] = distortionCoefficientNode[i];
  //     }
  //     calib.focalLength << focalLengthNode[0], focalLengthNode[1];
  //     calib.principalPoint << principalPointNode[0], principalPointNode[1];
  //     calib.distortionType = std::string((*it)["distortion_type"]);

  //     // parse additional stuff
  //     if((*it)["camera_type"].isString()){
  //       std::string camera_type = std::string((*it)["camera_type"]);
  //       if(camera_type.compare(0,4,"gray")==0) {
  //         calib.cameraType.isColour = false;
  //       } else {
  //         calib.cameraType.isColour = true;
  //       }
  //       if(camera_type.size()>=6){
  //         if(camera_type.compare(camera_type.size()-6,6,"+depth")==0) {
  //           calib.cameraType.depthType.isDepthCamera = true;
  //         } else {
  //           calib.cameraType.depthType.isDepthCamera = false;
  //         }
  //       }
  //     }
  //     if((*it)["slam_use"].isString()){
  //       std::string slam_use = std::string((*it)["slam_use"]);
  //       if(slam_use.compare(0,5,"okvis")==0) {
  //         calib.cameraType.isUsed = true;
  //       } else {
  //         calib.cameraType.isUsed = false;
  //       }
  //       if(slam_use.size()>=6){
  //         if(slam_use.compare(slam_use.size()-6,6,"-depth")==0) {
  //           calib.cameraType.depthType.createDepth = true;
  //         } else {
  //           calib.cameraType.depthType.createDepth = false;
  //         }
  //       }
  //       if(slam_use.size()>=8){
  //         if(slam_use.compare(slam_use.size()-8,8,"-virtual")==0) {
  //           calib.cameraType.depthType.createVirtual = true;
  //         } else {
  //           calib.cameraType.depthType.createVirtual = false;
  //         }
  //       }
  //     }
  //     if((*it)["sigma_pixels"].isReal()){
  //       calib.cameraType.depthType.sigmaPixels = ((*it)["sigma_pixels"]);
  //     }
  //     if((*it)["sigma_depth"].isReal()){
  //       calib.cameraType.depthType.sigmaPixels = ((*it)["sigma_depth"]);
  //     }

  //     calibrations.push_back(calib);
  //   }
  // }
  //return gotCalibration;
//}

}  // namespace okvis
