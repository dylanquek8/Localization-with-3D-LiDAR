# Localization-with-3D-LiDAR
This github repository contains the packages used for Localization with Robosense AIRY (3D LiDAR)

## Preparation of IMU data
### IMU calibration
A stationary bag file (calibration.db3) of the LiDAR is recorded for static calibration. imu_to_csv.py records imu data from calibration.db3 into a csv file. analyze_imu_noise.py reads the csv file and obtains the covariance for angular velocity and linear acceleration.

### Correcting IMU frame
<img width="675" height="599" alt="image" src="https://github.com/user-attachments/assets/df7e8676-c9df-452c-9c87-a552ccd4ea49" />.\
https://github.com/RoboSense-LiDAR/rslidar_sdk/issues/172 This issue raised in Robosense's github shows that the IMU frame is following REP-145 while LiDAR frame is following REP-103 (ENU).
```
  static bool output = true;
  if(output){
    output = false;
    RS_INFOL << "imu_calib rotation[x,y,z,w]:[" << this->device_info_.qx << ","
             << this->device_info_.qy << "," << this->device_info_.qz << ","
             << this->device_info_.qw << "]" << " translation[x,y,z]:["
             << this->device_info_.x << "," << this->device_info_.y << ","
             << this->device_info_.z << "]" << RS_REND;
```
Adding this into decoder_RSAIRY.hpp of robosense driver publishes the calibration quaternion and translation vectors from imu to lidar when launching the robosense driver.
### Convert to ROS REP-103
IMU data is used as input for robot_localization package and the package expects the data to be in ROS REP-103 format.  
imu103 package does this by changing the units of the imu data from g -> m/s^2 (linear acceleration) and deg/s -> rad/s (angular velocity) and publishing it. Covariance obtained from static calibration is also published in imu message.  
Using the calibration quaternion, it also rotates the imu data from the IMU frame to LiDAR frame (ENU).
### Publishing Orientation in IMU message
Robosense built-in IMU is 6DOF and thus it does not publish orientation. imu_filter_madgwick uses angular velocity published from imu to publish orientation. It also assumes input data is in ENU frame. Also publishes odom -> base_link tf

## Mapping
Map is obtained using SLAM package Direct-LiDAR-Inertial-Odometry.

## LiDAR Localization
lidar_localization_ros2 package is used for lidar localization to obtain pose for robot_localization input. IMU data is not used here, only LiDAR. Uses map obtained from SLAM (rs_warehouse_full.pcd). Publishes map -> odom tf.  
Change topics for lidar points in launch file
```
remappings = [('cloud','/rslidar_points')
```
## Robot_localization
Pose from lidar_localization is fused with orientation from imu data using robot_localization ekf package.  
IMU orientation, angular velocity and linear acceleration are all used in robot_localization to obtain best outcome.  
<img width="1595" height="960" alt="Screenshot from 2025-09-25 12-02-51" src="https://github.com/user-attachments/assets/f44717fd-d9a0-4abc-b072-374179913dc9" />  
Pose X: Filtered Odometry from robot localization (top) vs pose from lidar localization (bottom)  
<img width="1585" height="917" alt="Screenshot from 2025-09-25 12-05-17" src="https://github.com/user-attachments/assets/ddb9174c-d76e-4474-b209-d8326de0fb1e" />
Pose Y: Filtered Odometry from robot localization (top) vs pose from lidar localization (bottom)  
Data recorded from warehouse4.db3

## TF Tree
<img width="1826" height="923" alt="Screenshot from 2025-09-25 15-21-16" src="https://github.com/user-attachments/assets/260634bc-9499-4449-90e3-d830bbb63f72" />


## Steps to reproduce
Launch imu103 package
```
ros2 launch imu103 imu103.launch.py
```
Launch imu madgwick filter
```
ros2 launch imu_filter_madgwick imu_filter.launch.py
```
Launch lidar_localization
```
ros2 launch lidar_localization_ros2 lidar_localization.launch.py
```
Launch rviz and set initial pose
 <img width="1625" height="895" alt="Screenshot from 2025-09-25 14-33-22" src="https://github.com/user-attachments/assets/498a9200-ec72-40ed-b6ac-7a2258b8daa5" />

Launch robot_localization
```
ros2 launch robot_localization ekf.launch.py
```
