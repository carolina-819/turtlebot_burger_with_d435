#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/LaserScan.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <ros/console.h>
#include <iostream>
#include <fstream>
#include "../include/librealsense2/h/rs_types.h"
#include <image_transport/image_transport.h>

typedef struct {
    double fx, fy;
    double cx, cy;
} camera_parameters;

typedef struct {
    double avg;
    std::vector<cv::Point> pontos;
} zone;

camera_parameters camera_params;

rs2_intrinsics intriseco_depth;
rs2_intrinsics intriseco_color;
sensor_msgs::CameraInfo cam_info;
int ts = 2;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher pub_cloud_depth, pub_cloud_lidar;
std::string frame_color;
std::string frame_depth;
std::vector<cv::KeyPoint> keypoints;

cv::Mat depth_map;

std::ofstream MyFile;