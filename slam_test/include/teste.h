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
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/core/types.hpp>

double dx = 0.018;
double dy = 0.0175;
float tole = 0.5;

bool init;
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
std::vector<cv::KeyPoint> keypoints_old, keypoints_new;

cv::Mat img_old, img_new;
cv::Mat descriptors_old, descriptors_new;

cv::Mat depth_map;

std::ofstream MyFile;

const double sigma_lidar1 = 0.015; //em metros
const double sigma_lidar2 = 0.05; //em percentagem
const double sigma_vision = 0.02; //em percentagem

double recalc_z, recalc_x;