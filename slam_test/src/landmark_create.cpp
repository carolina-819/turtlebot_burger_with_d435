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
double dx = 0.018;
double dy = 0.0175;
float tole = 0.5;
void merge_callback(const sensor_msgs::PointCloud2ConstPtr &depth_msg, const sensor_msgs::LaserScanConstPtr& lidar_msg){
    pointcloud->clear();
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::PCLPointCloud2 pcl_depth;

    pcl_conversions::toPCL(*depth_msg, pcl_depth);
    
    pcl::fromPCLPointCloud2(pcl_depth, *pc_depth);
    int size_ = 0;
    
    for(int i = 0; i < pc_depth->points.size(); i++){
     /*   geometry_msgs::PointStamped stamped_in1;
        stamped_in1.point.x = pc_depth->at(i).x;
        stamped_in1.point.y = pc_depth->at(i).y;
        stamped_in1.point.z = pc_depth->at(i).z;
        geometry_msgs::PointStamped stamped_out1;
        // altera pontos da frame da pc para a frame do lidar
     //    listener->transformPoint("base_scan", stamped_in1, stamped_out1);
        // calcula bearing na frame do lidar
        double bearing = std::atan2(stamped_out1.point.y, stamped_out1.point.x); //este bearing ja esta na frame do lidar
       
        double bearing_lidar = bearing - M_PI;
        //descobre posição aproximada 
        int pos = bearing/(lidar_msg->angle_increment);
        double distance = lidar_msg->ranges[pos];
        // se o ponto naquele raio, ou um dos pontos ao lado, tem uma distancia parecida
        if((abs(distance - ( std::sqrt(std::pow(stamped_out1.point.x, 2) + std::pow(stamped_out1.point.y, 2)) )) < tole ) ||
            (abs(lidar_msg->ranges[pos + 1] - ( std::sqrt(std::pow(stamped_out1.point.x, 2) + std::pow(stamped_out1.point.y, 2)) )) < tole) ||
            (abs(lidar_msg->ranges[pos - 1] - ( std::sqrt(std::pow(stamped_out1.point.x, 2) + std::pow(stamped_out1.point.y, 2)) )) < tole) ) {
            
            std::cout << "encontrou correspondencia" << std::endl;
            geometry_msgs::PointStamped stamped_in2;
            geometry_msgs::PointStamped stamped_out2;

            pcl::PointXYZ p;
            // cria ponto na frame do lidar com as respetivas distancias tendo em conta os valores do lidar
            stamped_in2.point.x = distance * std::cos(bearing);
            stamped_in2.point.y = distance * std::sin(bearing);
            stamped_in2.point.z = 0;
            // transforma ponto nas coordenadas da camera
            listener->transformPoint("d435_color_optical_frame", stamped_in2, stamped_out2);
            p.x = stamped_out2.point.x;
            p.y = stamped_out2.point.y;
            p.z = stamped_out2.point.z;
            // publica esse ponto
            pointcloud->points.push_back(p);
            size_++;
        }*/
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

        if(distance > 0){
            std::cout << "encontrou correspondencia" << i << std::endl;
            pcl::PointXYZ p;
            
            // passa de volta para as coordenadas da frame de visao, desta vez com a informaçao do lidar, que a partida é mais reliable??
            p.y = ponto.y;
            p.z = (distance * std::cos(bearing_lidar)) + dx; 
            p.x = dy - (distance * std::sin(bearing_lidar));
            pointcloud->points.push_back(p);
            size_++;
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