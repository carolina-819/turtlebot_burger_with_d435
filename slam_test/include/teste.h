#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/CameraInfo.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>
#include <ros/console.h>

typedef struct {
    double fx, fy;
    double cx, cy;
} camera_parameters;

typedef struct {
    double avg;
    std::vector<cv::Point> pontos;
} zone;

camera_parameters camera_params;

sensor_msgs::CameraInfo cam_info;

pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);

cv::Mat depth_map;