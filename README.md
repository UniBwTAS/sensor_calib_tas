# An Extrinsic Sensor Calibration Framework for Precise Probabilistic Joint Calibration of Camera, LiDAR, and Radar

**sensor\_calib\_tas** provides:

- A ROS-based calibration tool for the 6D joint calibration of cameras, LiDAR sensors, and radar sensors
- An accompanying GUI for measurement recording, calibration, and verification
- ROS packages for detecting the calibration target using various sensors (including sparse and tilted LiDAR sensors and 4D high definition radar sensors)

**Author**: [Bianca Forkel](https://www.researchgate.net/profile/Bianca-Forkel)

![extrinsic_calibration](https://github.com/user-attachments/assets/d93b09c1-7f5d-4426-911b-b8f022845d67)

## Citation

If you use this code, please cite our **ITSC 2025** paper:

- B. Forkel, P. Berthold, and M. Maehlisch, “An Extrinsic Sensor Calibration Framework for Precise Probabilistic Joint Calibration of Camera, LiDAR, and Radar”, in Proceedings of IEEE International Conference on Intelligent Transportation Systems (ITSC), 2025.

```
@InProceedings{Forkel2025_ExtrinsicCalibration,
  author    = {Bianca Forkel AND Philipp Berthold AND Mirko Maehlisch},
  booktitle = {Proceedings of IEEE International Conference on Intelligent Transportation Systems (ITSC)},
  title     = {{An Extrinsic Sensor Calibration Framework for Precise Probabilistic Joint Calibration of Camera, LiDAR, and Radar}},
  year      = {2025}
}
```

## Abstract

Accurate sensor data fusion requires accurate knowledge of the sensors' relative mounting poses. To calibrate the sensor setup of autonomous vehicles, we propose a tool for the joint 6D extrinsic calibration of cameras, LiDAR sensors, and radar sensors. Our optimization-based method achieves high accuracy by probabilistically accounting for measurement uncertainties and employing an advanced calibration target. In addition to optimized visual markers and geometric features detectable by cameras and LiDAR sensors, the calibration target includes a radar Doppler simulator. This enables radar sensors to detect the calibration target with high accuracy and unambiguity. The proposed calibration tool is quantitatively evaluated on real-world data and released as an open-source ROS package.

## Calibration Target

Our calibration target builds upon that of [`velo2cam_calibration` v1.0](https://github.com/beltransen/velo2cam_calibration/tree/v1.0) and consists of two parts:

<img width="333" height="222" alt="hole_board" src="https://github.com/user-attachments/assets/b418eab9-01b5-432f-ab6a-4ce193f73ca6"/> <img width="333" height="222" alt="doppler_simulator" src="https://github.com/user-attachments/assets/d737aed1-875b-49b3-8981-75601538ef43" />

- A board with holes for detection in the LiDAR point cloud (blue reference points) and AprilTags for detection in the camera image (red reference points).
  Our specific board is 1.2m x 0.8m in size. The holes have a diameter of 24cm, and their centers are offset by 15cm in x- and y-direction from the center of the calibration board. The AprilTags belong to the 36h11 family and are extended by checkerboard corner for improved detection (see [`apriltags_tas`](https://github.com/UniBwTAS/apriltags_tas)). The AprilTags have an edge length of 22.8cm (excluding the checkerboard corners). The colored numbers show the respective IDs of all reference points.
  However, the calibration board is highly configurable in the calibration tool, such that you can build your own version of it.
- A radar Doppler simulator (in our case [MDS 77 by Heicks](https://www.heicks.de/wp-content/uploads/2024/09/BA-MDS-77-V001799.04_EN.pdf)) mounted on the board/its stand.

## Installation
Tested with Ubuntu 20.04 and ROS Noetic

Since you will need all your sensor data of the sensors to be calibrated in ROS, we assume an existing ROS installation.
Inside the `src` directory of your catkin workspace, clone this repository:

```
git clone https://github.com/UniBwTAS/sensor_calib_tas.git
cd sensor_calib_tas
git submodule update --init # Get git submodule apriltags_tas
git lfs pull  # Get demo files from git lfs
```

Get some dependencies from github:
```
cd ..
git clone https://github.com/catkin/catkin_simple.git
git clone https://github.com/ethz-asl/eigen_catkin.git
git clone https://github.com/1r0b1n0/librosqt.git
git clone https://github.com/UniBwTAS/ethernet_bridge.git
git clone https://github.com/UniBwTAS/tas_radar.git
```

Install system-wide dependencies:
```
sudo apt install libceres-dev=1.14.0-4ubuntu1.1
```

Install remaining dependencies using rosdep and ensure you are building in release mode:
```
rosdep install --from-paths . --ignore-src -r -y
catkin config --profile default --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Build the required packages:
```
catkin build extrinsic_sensor_calib apriltags_tas lidar_reference_point_detection radar_target_detection
```

## Demo

You can try the calibration tool on demo data using the provided demo launch file:
```
roslaunch extrinsic_sensor_calib demo.launch
```
This command plays a rosbag and starts the calibration tool together with its GUI inside *rqt* as well as *rviz*.

### Steps to explore a typical calibration workflow

1. **Load prerecorded measurements**
In the *rqt* GUI, click on the open file symbol (top right corner of the *Extrinsic Sensor Calib* plugin) and load `demo/measurements_mucar3.bin`

2. **Inspect individual measurements**
In the *Measurements* tab, use the drop down menu to browse through the single measurements.
The debug images in *rqt* and the markers in *rviz* then show
- the selected measurement (yellow), and
- the predicted measurement based on the estimated board pose (green).
Visualize all recorded board poses by enabling all *TF frames* in *rviz* or the option `show_all_measurements` in *Dynamic Reconfigure* (to visualize all detected marker corners in the debug camera images).

3. **View detection results in recorded scene**
The live outputs of the calibration target detection algorithms appear as
- red markers in *rviz* (LiDAR, radar)
- colored boxes in the debug images of the AprilTag detector (camera)

4. **Add new measurement**
Click **Measure new board pose** in the *rqt* GUI to add a new measurement from the current detections. To average multiple detections of the same board pose, click **Amend to last board pose** repeatedly.

5. **Run the calibration**
Click **Solve** to perform the calibration.
After solving, the poses in the tabs *Sensor Poses* and *Board Poses* change, and the RMSE values in the *Verification* tab should decrease. The green predicted measurements should have moved closer to the yellow observed measurements. Inspecting the *tf* frames in *rviz*, for every sensor, the `_calibrated` frame now differs from the original sensor frame.

6. **Preview the optimized poses**
Click **Preview** to publish the optimized sensor poses to *tf*. *rviz* will then use the updated poses to visualize the sensor data (including the projection of markers or point clouds into images).

7. **Get calibrated sensor poses**
In the *Sensor Poses* tab, right-click on any sensor to copy the calibrated pose to the clipboard.

8. **Configure optimization**
Reset individual sensor or board poses by right-clicking on their entry, or reset everything using the **Reset** button.
Exclude single measurement residuals, sensor poses (translation and/or rotation), or board poses from the optimization using the checkboxes in the respective tabs.

9. **Save everything to file**
Click the disk icon in the top right corner of the *Extrinsic Sensor Calib* GUI to save all measurements and poses to a binary file. You can later resume your calibration by loading this file into the GUI.
It is also possible to combine measurements from multiple recordings by loading one file after another.

10. **Explore the GUI**
Hover over the buttons and tabs in the GUI to see tooltips describing their functionality. Find some more debug options and configuration parameters for the reference point detectors in *Dynamic Reconfigure*.

## GUI

<img width="1920" height="1200" alt="rqt" src="https://github.com/user-attachments/assets/73dd7565-62e1-4cbd-bce5-d65fb4347469" />
<img width="1920" height="1164" alt="rviz" src="https://github.com/user-attachments/assets/b9afab81-76aa-4e36-93b5-d78bb8e0be63" />

## Package Structure

This repository contains six ROS packages:
- `extrinsic_sensor_calib`: Main calibration tool
- `rqt_sensor_calib`: Calibration GUI as rqt plugin
- `apriltag_detection`: Detects AprilTags in camera images, see [original repository](https://github.com/UniBwTAS/apriltags_tas)
- `lidar_reference_point_detection`: Detects hole board in LiDAR point cloud
- `radar_target_detection`: Detects Doppler simulator in radar detections
- `sensor_calib_msgs`: ROS messages for detected reference points and for communication with the GUI

You can easily add your own detection algorithms tailored for your custom calibration target or your specific sensor by publishing the corresponding messages from sensor_calib_msgs and configuring the topic in the launch file.

## Sensor Setup Configuration

To configure your own sensor setup, create a launch file based on the example in `extrinsic_sensor_calib/demo/calibrate_mucar3.launch`:
- Load board config
  ```
  <rosparam command="load" file="$(find extrinsic_sensor_calib)/config/board_config.yaml" ns="sensor_calib/board_config"/>
  ```
- Launch one reference point detector per camera and LiDAR sensor, as well as one reference point detector for all radar sensors. Specify sensor names, input data topics (for cameras: without last `/image`), and, for every camera, suffix of the image topic to use, e.g., `raw`, `mono`, `color`, `rect_mono`, `rect_color`. For the LiDAR sensor, specify a ROI to which the point cloud is cropped. It can be adapted during runtime using *Dynamic Reconfigure* (in the nodes `pass_through_*`).
  ```
  <!-- AprilTag detectors -->
  <include file="$(find extrinsic_sensor_calib)/launch/camera_apriltag_detection.launch">
    <arg name="name" value="surround_front"/>
    <arg name="topic" value="/sensor/camera/surround/front"/>
    <arg name="color_mode" value="rect_color"/>
  </include>
  <include file="$(find extrinsic_sensor_calib)/launch/camera_apriltag_detection.launch">
    <arg name="name" value="surround_right"/>
    <arg name="topic" value="/sensor/camera/surround/right"/>
    <arg name="color_mode" value="color"/>
  </include>

  <!-- LiDAR hole detector -->
  <include file="$(find extrinsic_sensor_calib)/launch/lidar_hole_pattern_detection.launch">
    <arg name="name" value="vls128"/>
    <arg name="topic" value="/sensor/lidar/vls128_roof/velodyne_points"/>
    <arg name="min_x" value="5"/>
    <arg name="max_x" value="20"/>
    <arg name="min_y" value="-10"/>
    <arg name="max_y" value="10"/>
    <arg name="min_z" value="-2"/>
    <arg name="max_z" value="3"/>
  </include>

  <!-- Radar target detector -->
  <include file="$(find extrinsic_sensor_calib)/launch/radar_doppler_simulator_detection.launch">
    <arg name="name" value="umrr"/>
    <arg name="topic" value="/sensor/radar/umrr/detections"/>
  </include>
  ```
- Launch calibration node and specify
  - reference_frame_id (frame id of the sensor to calibrate all other sensors to)
  - cameras with frame_id, detector_topic, rectified (true/false), sigma (expected measurement noise)
  - lidars with frame_id, detector_topic, sigma
  - radars with frame_id, detector_topic, sigma_range, sigma_azimuth, sigma_elevation
  ```
  <node pkg="extrinsic_sensor_calib" type="extrinsic_sensor_calib" name="extrinsic_sensor_calib" ns="sensor_calib" output="screen">
    <remap from="~board_config/boards" to="/sensor_calib/board_config/boards"/>
    <rosparam>
      reference_frame: sensor/lidar/vls128_roof
      lidars:
          - {frame_id: sensor/lidar/vls128_roof, detector_topic: /sensor_calib/hole_pattern_detector_vls128/detections, sigma: 0.01}
      cameras:
          - {frame_id: sensor/camera/surround/front, detector_topic: /sensor_calib/apriltag_detector_surround_front/detections, rectified: true, sigma: 0.2}
          - {frame_id: sensor/camera/surround/right, detector_topic: /sensor_calib/apriltag_detector_surround_right/detections, rectified: false, sigma: 0.2}
      radars:
          - {frame_id: sensor/radar/umrr/32, detector_topic: /sensor_calib/doppler_simulator_detector_umrr/umrr/32, sigma_range: 0.1, sigma_azimuth: 0.5, sigma_elevation: 9}
          - {frame_id: sensor/radar/umrr/35, detector_topic: /sensor_calib/doppler_simulator_detector_umrr/umrr/35, sigma_range: 0.1, sigma_azimuth: 0.5, sigma_elevation: 9}
    </rosparam>
  </node>
  ```
- Start *rqt* and *rviz* for visualization
  ```
  <node type="rviz" name="rviz" pkg="rviz" args="-d $(find extrinsic_sensor_calib)/config/extrinsic_sensor_calib.rviz" ns="sensor_calib"/>
  <node type="rqt_gui" name="rqt_gui" pkg="rqt_gui" args="--perspective-file $(find extrinsic_sensor_calib)/config/extrinsic_sensor_calib.perspective" ns="sensor_calib"/>
  ```
  
## Custom Calibration Target Configuration

To configure a custom calibration target, modify `extrinsic_sensor_calib/config/board_config.yaml`.
For each calibration target add:
- name
- width
- height
- layout (AprilTags, if applicable) with id, size, x, y, z, (qx, qy, qz, qw)
- lidar_reference_points (if applicable) with id, radius, x, y, z
- radar_targets (if applicable) with id, speed, x, y, z

We assume the origin of the board coordinate system in its center, with the x axis pointing right, the y axis pointing up, and the z axis pointing towards the vehicle.

```
boards:
  [
    {
      name: 'hole_board',
      width: 1.2,
      height: 0.8,
      layout: # AprilTags
        [
          { id: 200, size: 0.228, x: -0.458, y: +0.258, z: 0.0 },
          { id: 201, size: 0.228, x: +0.458, y: +0.257, z: 0.0 },
          { id: 202, size: 0.228, x: -0.458, y: -0.256, z: 0.0 },
          { id: 203, size: 0.228, x: +0.458, y: -0.256, z: 0.0 },
        ],
      lidar_reference_points:
        [
          { id: 0, radius: 0.12, x: -0.15, y: +0.15, z: 0.0 }, # top_left
          { id: 1, radius: 0.12, x: +0.15, y: +0.15, z: 0.0 }, # top_right
          { id: 2, radius: 0.12, x: +0.15, y: -0.15, z: 0.0 }, # bot_right
          { id: 3, radius: 0.12, x: -0.15, y: -0.15, z: 0.0 }, # bot_left
        ],
      radar_targets:
        [
          { id: -2000, speed: 4.18, x: 0.009, y: -1.169, z: -0.1144 } # id = rpm of doppler simulator
        ]
    },
  ]
```
