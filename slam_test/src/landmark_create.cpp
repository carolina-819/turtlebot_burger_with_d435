#include "../include/teste.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>

ros::Publisher landmarks_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_depth(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_lidar(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

void merge_callback(const sensor_msgs::PointCloud2ConstPtr &depth_msg, const sensor_msgs::PointCloud2ConstPtr &lidar_msg){

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PCLPointCloud2 pcl_depth, pcl_lidar;

    pcl_conversions::toPCL(*depth_msg, pcl_depth);
    pcl_conversions::toPCL(*lidar_msg, pcl_lidar);
    
    pcl::fromPCLPointCloud2(pcl_depth, *pc_depth);
    pcl::fromPCLPointCloud2(pcl_lidar, *pc_lidar);
    int size_ = 0;
    for(int i = 0; i < pc_depth->points.size(); i++){
        pointcloud->points.push_back(pc_depth->at(i));
        size_++;
    }
    
    for(int i = 0; i < pc_lidar->points.size(); i++){
        pointcloud->points.push_back(pc_lidar->at(i));
        size_++;
    }
    pointcloud->header.frame_id = "d435_color_optical_frame";
    pcl::toROSMsg(*pointcloud.get(), cloud_msg);
     
    landmarks_pub.publish(cloud_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
   
    landmarks_pub = nh.advertise<sensor_msgs::PointCloud2> ("landmarks", 1);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_depth_sub(nh, "points_depth", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_lidar_sub(nh, "points_lidar", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_depth_sub, cloud_lidar_sub);

    sync.registerCallback(boost::bind(&merge_callback, _1, _2));
     std::cout << "AAAA" << std::endl;
    
    while (ros::ok())
    {
        ros::spinOnce();
    }

    return 0;
}