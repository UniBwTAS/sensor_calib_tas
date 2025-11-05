/*
 * Copyright (c) 2025, Perception for Autonomous Driving Lab,
 * Institute for Autonomous Driving, Universität der Bundeswehr München
 * All rights reserved.
 *
 * Author: Bianca Forkel
 *
 * This file is part of the sensor_calib_tas package.
 * Licensed under the BSD 3-Clause License.
 * See the LICENSE file in the project root for full license information.
 */

#pragma once

#include <ceres/ceres.h>
#include <opencv2/viz/types.hpp>
#include <pcl/point_cloud.h>

#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <apriltags_msgs/AprilTagDetections.h>
#include <sensor_calib_msgs/CartesianPointMeasurements.h>
#include <sensor_calib_msgs/DetectionStatus.h>
#include <sensor_calib_msgs/MeasurementIndex.h>
#include <sensor_calib_msgs/OptimizeSensorPoses.h>
#include <sensor_calib_msgs/PolarPointMeasurements.h>
#include <sensor_calib_msgs/Reset.h>
#include <sensor_calib_msgs/UseResiduals.h>

#include <extrinsic_sensor_calib/ExtrinsicSensorCalibConfig.h>

#include "boost_serialization.hpp"
#include "types.h"

class SensorCalib
{
  public:
    SensorCalib(ros::NodeHandle& nh);

    void reconfigure(extrinsic_sensor_calib::ExtrinsicSensorCalibConfig& config, uint32_t level);

    void callbackAprilTagDetections(const apriltags_msgs::AprilTagDetections::ConstPtr& msg) noexcept;
    void callbackLiDARTargetDetections(const sensor_calib_msgs::CartesianPointMeasurements::ConstPtr& msg) noexcept;
    void callbackRadarTargetDetections(const sensor_calib_msgs::PolarPointMeasurements::ConstPtr& msg) noexcept;

    void measureRequest(const std_msgs::Bool::ConstPtr& new_board_pose) noexcept;
    void removeLastMeasurementRequest(const std_msgs::Empty::ConstPtr&) noexcept;
    void solveRequest(const std_msgs::Empty::ConstPtr&) noexcept;
    void verifyRequest(const std_msgs::Empty::ConstPtr&) noexcept;
    void resetSensorPoseRequest(const sensor_calib_msgs::Reset::ConstPtr&) noexcept;
    void resetBoardPoseRequest(const sensor_calib_msgs::Reset::ConstPtr&) noexcept;
    void previewRequest(const std_msgs::Bool::ConstPtr&) noexcept;
    void updateUsedMeasurements(const sensor_calib_msgs::UseResiduals::ConstPtr&) noexcept;
    void updateOptimizedSensorPoses(const sensor_calib_msgs::OptimizeSensorPoses::ConstPtr&) noexcept;
    void updateVisualizedMeasurement(const sensor_calib_msgs::MeasurementIndex::ConstPtr&) noexcept;
    void saveToFileRequest(const std_msgs::String::ConstPtr& msg) const noexcept;
    void loadFromFileRequest(const std_msgs::String::ConstPtr& msg) noexcept;

    void publishResults() noexcept;

    void publishStatusMsg(const std::string& status_msg = "Idle") const noexcept;

    template<typename Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar & lidar_reference_point_detector_topics_;
        ar & apriltag_detector_topics_;
        ar & radar_target_detector_topics_;
        ar & reference_frame_id_;
        ar & calibration_boards_;
        ar & board_poses_;
        ar & initial_board_poses_;
        ar & sensor_poses_;
        ar & initial_sensor_poses_;
        ar & lidar_measurements_;
        ar & camera_measurements_;
        ar & radar_measurements_;
        ar & num_board_poses_;
        ar & use_measurement_;
        ar & optimize_sensor_pose_;
        ar & measurement_uncertainties_;
        ar & camera_intrinsics_;
        ar & camera_topics_;
        ar & lidar_frame_ids_;
        ar & camera_frame_ids_;
        ar & radar_frame_ids_;

        if (config_.save_camera_images)
        {
            ar & camera_measurement_images_;
        }
    }

    std::vector<std::string> lidar_reference_point_detector_topics_;
    std::vector<std::string> apriltag_detector_topics_;
    std::vector<std::string> radar_target_detector_topics_;

  private:
    void initCameraPublishers() noexcept;

    bool measureLiDARTargets() noexcept;
    bool measureAprilTags() noexcept;
    bool measureRadarTargets() noexcept;
    bool fillUpMeasurements(const int num_measurements_for_board_pose) noexcept;

    void initBoardPoses() noexcept;
    std::optional<Pose>
    getBoardPoseFromCameraMeasurement(const BoardID& board_id,
                                      const FrameID& camera_frame_id,
                                      const CameraBoardMeasurement& tag_measurements) const noexcept;
    std::optional<Pose>
    getBoardPoseFromLiDARMeasurement(const BoardID& board_id,
                                     const LiDARBoardMeasurement& lidar_target_measurements) const noexcept;
    std::optional<Pose>
    getBoardPoseFromRadarMeasurement(const BoardID& board_id,
                                     const RadarBoardMeasurement& radar_target_measurements) const noexcept;
    void deleteLastMeasurement() noexcept;

    void publishSensorPosesToTf(const std::string& suffix, const bool initial = false) noexcept;
    void visualizeMeasurements(const cv::viz::Color& measurement_color) noexcept;
    void visualizeExpectedMeasurements(const cv::viz::Color& prediction_color) noexcept;
    void visualizeImageMeasurements(const cv::viz::Color& measurement_color,
                                    const cv::viz::Color& prediction_color,
                                    const cv::viz::Color& accumulated_color,
                                    const cv::viz::Color& error_color) noexcept;

    // Helpers
    Pose getPoseFromTF(const std::string& from_frame_id, const std::string& to_frame_id) const noexcept;
    Pose convertToReferenceFrame(const Pose& pose, const FrameID& sensor_frame_id) const noexcept;

    // Adapted from apriltag_ros
    void parseBoardConfig(const XmlRpc::XmlRpcValue& board_config);
    double xmlRpcGetDoubleWithDefault(const XmlRpc::XmlRpcValue& xmlValue,
                                      const std::string& field,
                                      const double defaultValue) const;

    void initSensorPoses() noexcept;
    void resetSensorPosesToZero() noexcept;
    void initIntrinsics() noexcept;

    void measure(const bool new_board_pose) noexcept;
    void calibrate(const bool solve) noexcept;

    bool isCamera(const FrameID& frame_id) const noexcept;
    bool isLiDAR(const FrameID& frame_id) const noexcept;
    bool isRadar(const FrameID& frame_id) const noexcept;

    std::string timeIndexStr(const MeasurementIndex idx, const size_t id = -1) const noexcept;

    // The tf frame id of the reference (sensor). Usually velodyne LiDAR
    FrameID reference_frame_id_;

    // Board configurations from yaml
    std::map<BoardID, BoardConfig> calibration_boards_;

    // Board poses relative to the reference frame
    std::map<BoardID, std::map<MeasurementIndex, Pose>> board_poses_;
    std::map<BoardID, std::map<MeasurementIndex, Pose>> initial_board_poses_;

    // Sensor poses relative to the reference frame
    std::map<std::pair<FrameID, FrameID>, Pose> sensor_poses_;
    std::map<std::pair<FrameID, FrameID>, Pose> initial_sensor_poses_;

    // LiDAR measurements
    // frame_id of the LiDAR sensor, index k of the measurement time step, vector (from amend) of board measurements
    std::map<FrameID, std::map<MeasurementIndex, std::vector<LiDARMeasurement>>> lidar_measurements_;

    // Camera measurements
    // frame_id of the camera, index k of the measurement time step, vector (from amend) of board measurements
    std::map<FrameID, std::map<MeasurementIndex, std::vector<CameraMeasurement>>> camera_measurements_;

    // Radar measurements
    // frame_id of the radar sensor, index k of the measurement time step, vector (from amend) of board measurements
    std::map<FrameID, std::map<MeasurementIndex, std::vector<RadarMeasurement>>> radar_measurements_;

    std::map<FrameID, apriltags_msgs::AprilTagDetections::ConstPtr> current_apriltag_detections_;
    std::map<FrameID, sensor_calib_msgs::CartesianPointMeasurements::ConstPtr>
        current_lidar_reference_point_detections_;
    std::map<FrameID, sensor_calib_msgs::PolarPointMeasurements::ConstPtr> current_radar_target_detections_;

    std::map<FrameID, std::map<MeasurementIndex, std::vector<cv::Mat>>> camera_measurement_images_;

    int num_board_poses_{0};

    std::map<std::pair<std::string, std::string>, bool> use_measurement_;
    std::map<std::pair<std::string, std::string>, std::pair<bool, bool>> optimize_sensor_pose_;

    std::map<FrameID, Uncertainties> measurement_uncertainties_;

    std::map<FrameID, CameraIntrinsics> camera_intrinsics_;

    std::vector<std::string> camera_topics_;

    std::vector<FrameID> lidar_frame_ids_;
    std::vector<FrameID> camera_frame_ids_;
    std::vector<FrameID> radar_frame_ids_;

    std::map<FrameID, sensor_calib_msgs::DetectionStatus> detection_stati_;

    bool preview_{false};

    int measurement_idx_to_debug_{-1};
    int measurement_id_to_debug_{-1};

    extrinsic_sensor_calib::ExtrinsicSensorCalibConfig config_;
    bool initialized_{false};

    bool sensor_poses_changed_ = true;
    bool board_poses_changed_ = true;
    bool measurements_changed_ = true;

    ros::Publisher sensor_poses_pub_;
    ros::Publisher board_poses_pub_;
    ros::Publisher residuals_pub_;
    ros::Publisher overall_residuals_pub_;
    ros::Publisher status_pub_;
    ros::Publisher marker_pub_;
    std::map<FrameID, ros::Publisher> debug_image_pub_;
    ros::Publisher detection_status_pub_;
    ros::Publisher measurement_indices_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_{tf_buffer_};

    tf2_ros::StaticTransformBroadcaster static_transform_broadcaster_;

    ros::NodeHandle nh_;
};

class CeresIterationCallback : public ceres::IterationCallback
{
  public:
    CeresIterationCallback(SensorCalib& extrinsic_sensor_calib) : camera_calib_(extrinsic_sensor_calib)
    {
    }

    virtual ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
    {
        camera_calib_.publishStatusMsg("Optimizing... current iteration: " + std::to_string(summary.iteration) +
                                       ", current cost: " + std::to_string(summary.cost));
        return ceres::SOLVER_CONTINUE;
    }

  private:
    SensorCalib& camera_calib_;
};