#include "../include/teste.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>

ros::Publisher landmarks_pub;
//tf::TransformListener *listener;
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_depth(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pc_lidar(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);

void merge_callback(const sensor_msgs::PointCloud2ConstPtr &depth_msg, const sensor_msgs::LaserScanConstPtr& lidar_msg){
    pointcloud->clear();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PCLPointCloud2 pcl_depth;

    pcl_conversions::toPCL(*depth_msg, pcl_depth);
    
    pcl::fromPCLPointCloud2(pcl_depth, *pc_depth);
    int size_ = 0;
    
    for(int i = 0; i < pc_depth->points.size(); i++){
        // altera pontos da frame da pc para a frame do lidar
     //    listener->transformPoint("base_scan", stamped_in1, stamped_out1);
        // calcula bearing na frame do lidar
        
        //se no lidar nao houver nada com um z parecido, descarta ponto
        //se houver, poe esse z na pointcloud, LANDMARK
        
         // procura no scan pontos com bearing parecido
         // filtra pontos, mas so quando esta muito perto dos objetos e quando o carro se mexe devagar
         pcl::PointXYZ ponto = pc_depth->at(i);
         // passa para a frame da camara
         double x_lidar = ponto.z - dx;
         double y_lidar = dy - ponto.x;
         double bearing_lidar = std::atan2(y_lidar, x_lidar); // bearing aproximado
        
        int pos = bearing_lidar/(lidar_msg->angle_increment);
        double distance = -1;
        if((abs(lidar_msg->ranges[pos] - ( std::sqrt(std::pow(x_lidar, 2) + std::pow(y_lidar, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos] > 0)) ) {
            distance = lidar_msg->ranges[pos];
        }
        else if((abs(lidar_msg->ranges[pos + 1] - ( std::sqrt(std::pow(x_lidar, 2) + std::pow(y_lidar, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos + 1] > 0)) ) {
            distance = lidar_msg->ranges[pos + 1];
        }else if((abs(lidar_msg->ranges[pos -1] - ( std::sqrt(std::pow(x_lidar, 2) + std::pow(y_lidar, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos - 1] > 0)) ) {
            distance = lidar_msg->ranges[pos - 1];
        }

        if(distance > 0 || (lidar_msg->ranges[pos] != lidar_msg->ranges[pos])){
           // std::cout << "encontrou correspondencia" << i << std::endl;
            pcl::PointXYZ p;
            if(!lidar_msg->ranges[pos]){
                std::cout << "e um menino (a distancia e infinita)" << lidar_msg->ranges[pos] << std::endl;
                pointcloud->points.push_back(ponto);
                size_++;
            }else{
                 // passa de volta para as coordenadas da frame de visao, desta vez com a informaçao do lidar, que a partida é mais reliable??
                p.y = ponto.y;
                p.z = (distance * std::cos(bearing_lidar)) + dx; 
                p.x = dy - (distance * std::sin(bearing_lidar));
                pointcloud->points.push_back(p);
                size_++;
            }
        }
            
            
    }
    
  /*  for(int i = 0; i < pc_lidar->points.size(); i++){
        pointcloud->points.push_back(pc_lidar->at(i));
        size_++;
    }*/
    pointcloud->header.frame_id = "d435_color_optical_frame";
    pcl::toROSMsg(*pointcloud.get(), cloud_msg);
     
    landmarks_pub.publish(cloud_msg);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle nh;
    
    
    //mea.range_ * std::cos(mea.bearing_ + mu_(2))
    // transforma pontos do lidar nos pontos da camara
    
 /*   tf::StampedTransform transform;
     tf2_ros::Buffer tfBuffer;
     tf2_ros::TransformListener tfListener(tfBuffer);
     geometry_msgs::TransformStamped transformStamped;
    listener = new tf::TransformListener();*/
    landmarks_pub = nh.advertise<sensor_msgs::PointCloud2> ("landmarks", 1);

    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_depth_sub(nh, "points_depth", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> cloud_lidar_sub(nh, "scan", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::LaserScan> MySyncPolicy;
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_depth_sub, cloud_lidar_sub);

    sync.registerCallback(boost::bind(&merge_callback, _1, _2));
     std::cout << "AAAA" << std::endl;
    
    while (ros::ok())
    {
        ros::Time t = ros::Time(0);
      /*  try{
         transformStamped = tfBuffer.lookupTransform("turtle2", "turtle1", ros::Time(0));
        }catch (tf2::TransformException &ex) {
         ROS_WARN("%s",ex.what());
         ros::Duration(1.0).sleep();
         continue;
       }*/
        
        ros::spinOnce();
    }

    return 0;
}