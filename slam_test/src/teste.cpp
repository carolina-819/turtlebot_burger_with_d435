#include "../include/teste.h"
#include "../include/librealsense2/rsutil.h"
#include "../include/librealsense2/h/rs_types.h"
#include "../include/librealsense2/h/rs_frame.h"
#include <iostream>
#include <fstream>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <opencv2/features2d.hpp>
using namespace cv;

void show_wait_destroy(const char* winname, Mat img);
void check_distance(std::vector<zone> &zonas, Point ponto, float distancia);
float check_coplanar(pcl::PointXYZ p1, pcl::PointXYZ p2, pcl::PointXYZ p3, pcl::PointXYZ p4){
    float x1 = p1.x;
    float y1 = p1.y;
    float z1 = p1.z;
    float x2 = p2.x;
    float y2 = p2.y;
    float z2 = p2.z;
    float x3 = p3.x;
    float y3 = p3.y;
    float z3 = p3.z;
    float x4 = p4.x;
    float y4 = p4.y;
    float z4 = p4.z;
    
    float a1 = x2 - x1 ;
    float b1 = y2 - y1 ;
    float c1 = z2 - z1 ;
    float a2 = x3 - x1 ;
    float b2 = y3 - y1 ;
    float c2 = z3 - z1 ;
    float a = b1 * c2 - b2 * c1 ;
    float b = a2 * c1 - a1 * c2 ;
    float c = a1 * b2 - b1 * a2 ;
    float d = (- a * x1 - b * y1 - c * z1) ;
       
    // equation of plane is: a*x + b*y + c*z = 0 #
       
    // checking if the 4th point satisfies
    // the above equation
    float result = (a * x4 + b * y4 + c * z4 + d);
    return result;

}


void cb_pcl(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PCLPointCloud2 pcl_toda;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_toda(new pcl::PointCloud<pcl::PointXYZ>);
    if(msg){
        pcl_conversions::toPCL(*msg, pcl_toda);
        pcl::fromPCLPointCloud2(pcl_toda, *pc_toda);

        std::cout << " tamanho recebido " << pc_toda->points.size() << std::endl;
   
    }
   
}
void get_depth_camera_info()
{
    boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo;
    do
    {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/d435/depth/camera_info", ros::Duration(5));
        ROS_WARN_STREAM("do");
        if (sharedCameraInfo != NULL)
        {
            cam_info = *sharedCameraInfo;
            intriseco_depth.ppx = cam_info.K[2];
            intriseco_depth.ppy = cam_info.K[5];
            intriseco_depth.fy = cam_info.K[4];
            intriseco_depth.fx = cam_info.K[0];
            intriseco_depth.height = cam_info.height;
            intriseco_depth.width = cam_info.width;
            frame_depth = cam_info.header.frame_id;
            if (cam_info.distortion_model == "plumb_bob") {
                intriseco_depth.model =  RS2_DISTORTION_BROWN_CONRADY;
            }else if  (cam_info.distortion_model == "equidistant") {
                intriseco_depth.model = RS2_DISTORTION_KANNALA_BRANDT4;
            }
            
            ROS_WARN_STREAM("Width = " << cam_info.width << " Height = " << cam_info.height);
        }
        else
        {
            ROS_ERROR("Couldn't get left camera info! Trying again...");
            ros::Duration(1.0).sleep();
        }
    } while (sharedCameraInfo == NULL);
    ROS_WARN_STREAM("done"); // esta preso no do
}

void get_color_camera_info()
{
    boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo;
    do
    {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/d435/color/camera_info", ros::Duration(5));
        ROS_WARN_STREAM("do");
        if (sharedCameraInfo != NULL)
        {
            cam_info = *sharedCameraInfo;
            intriseco_color.ppx = cam_info.K[2];
            intriseco_color.ppy = cam_info.K[5];
            intriseco_color.fy = cam_info.K[4];
            intriseco_color.fx = cam_info.K[0];
            intriseco_color.height = cam_info.height;
            intriseco_color.width = cam_info.width;
            frame_color = cam_info.header.frame_id;
            if (cam_info.distortion_model == "plumb_bob") {
                intriseco_color.model =  RS2_DISTORTION_BROWN_CONRADY;
            }else if  (cam_info.distortion_model == "equidistant") {
                intriseco_color.model = RS2_DISTORTION_KANNALA_BRANDT4;
            }
            
        }
        else
        {
            ROS_ERROR("Couldn't get image camera info! Trying again...");
            ros::Duration(1.0).sleep();
        }
    } while (sharedCameraInfo == NULL);
    ROS_WARN_STREAM("done"); 
}

void cb_align(const sensor_msgs::ImageConstPtr& msg)
{
    rs2_intrinsics intriseco = intriseco_color;
    cv_bridge::CvImagePtr depth_image_ptr;
    
    try
    {
        depth_image_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
   
    Mat img = depth_image_ptr->image;
    float w = intriseco.width;
    float h =  intriseco.height;

    Mat graymat(h, w, CV_8UC1);
    

    double minVal, maxVal;
    minMaxLoc(img, &minVal, &maxVal);
   // std::cout << "min: " << minVal << " max: " << maxVal << std::endl;

    Mat x_coord(intriseco.height, intriseco.width, CV_32F);
    Mat y_coord(intriseco.height, intriseco.width, CV_32F);
    Mat z_coord(intriseco.height, intriseco.width, CV_32F);
    
    for(u_int16_t i = 0; i < h; i++){ //goes through rows
        for (u_int16_t j = 0; j < w; j++){ //goes through columns
            float pixel[2] = {i, j};
            
            float point[3];
        //    rs2_deproject_pixel_to_point( point, &intriseco, pixel, img.at<u_int16_t>(i, j));
            graymat.at<unsigned char>(i, j) = uchar ((img.at<u_int16_t>(i, j) * 255)/maxVal);
         //   std::cout << "checkpoint " << point[0] << " " << point[1] << " " << point[2] << "depth " << img.at<u_int16_t>(i, j) << std::endl;
         //   std::cout << "valor " << img.at<u_int16_t>(i, j) << std::endl;
         /*   x_coord.at<float>(i, j) =  point[1] * 0.001;
            y_coord.at<float>(i, j) = (point[0])* 0.001;
            z_coord.at<float>(i, j) = point[2]* 0.001;
*/

            x_coord.at<float>(i, j) = ((j - intriseco.ppx)/(intriseco.fx)) * img.at<u_int16_t>(i, j) * 0.001;
            y_coord.at<float>(i, j) = ((i - intriseco.ppy)/(intriseco.fy)) * img.at<u_int16_t>(i, j) * 0.001;
            z_coord.at<float>(i, j) = img.at<u_int16_t>(i, j) * 0.001;
            
        }
    }
    
    Mat im_with_kp;
    cv::drawKeypoints( graymat, keypoints, im_with_kp, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );
    
    cv::imshow("image grey", im_with_kp);
    cv::waitKey(1);

    //check for coplanarity and filter keypoints
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud->header.frame_id = "d435_color_optical_frame";
    
    int size_pcl = 0;
    for (int i = 0; i < keypoints.size(); i++){
        pcl::PointXYZ p;
        p.z = z_coord.at<float>(keypoints[i].pt.y, keypoints[i].pt.x);
        p.y = y_coord.at<float>(keypoints[i].pt.y, keypoints[i].pt.x);
        p.x = x_coord.at<float>(keypoints[i].pt.y, keypoints[i].pt.x);
      /*  
        pcl::PointXYZ p1, p2, p3;
        p1.z = z_coord.at<float>(keypoints[i].pt.y - ts, keypoints[i].pt.x + ts);
        p1.y = y_coord.at<float>(keypoints[i].pt.y - ts, keypoints[i].pt.x + ts);
        p1.x = x_coord.at<float>(keypoints[i].pt.y - ts, keypoints[i].pt.x + ts);

        p2.z = z_coord.at<float>(keypoints[i].pt.y - ts, keypoints[i].pt.x - ts);
        p2.y = y_coord.at<float>(keypoints[i].pt.y - ts, keypoints[i].pt.x - ts);
        p2.x = x_coord.at<float>(keypoints[i].pt.y - ts, keypoints[i].pt.x - ts);

        p3.z = z_coord.at<float>(keypoints[i].pt.y + ts, keypoints[i].pt.x);
        p3.y = y_coord.at<float>(keypoints[i].pt.y + ts, keypoints[i].pt.x);
        p3.x = x_coord.at<float>(keypoints[i].pt.y + ts, keypoints[i].pt.x);

        float result = check_coplanar(p, p1, p2, p3);
        if(abs(result) > 1e-09){ //aumentar este valor
            std::cout << "resultado coplanar " << result << std::endl;*/
            pointcloud->points.push_back(p);
            size_pcl++;
        //}
        

      //  pointcloud->points.push_back(p1);
      //  pointcloud->points.push_back(p2);
      //  pointcloud->points.push_back(p3);
    }
    pointcloud->width = size_pcl;
    pointcloud->height = 1;
    ros::Time time_st = ros::Time::now ();
    pointcloud->header.stamp = time_st.toNSec()/1e3;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pointcloud.get(), cloud_msg);
    pub_cloud_depth.publish(cloud_msg);
    
}
void cb_rgb(const sensor_msgs::Image &msg){ //detectar objetos, meter bounding boxes nos objetos

    cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(msg);
   
    Mat src = depth_image_ptr->image.clone();
    Mat greyMat;
    cv::cvtColor(src, greyMat, COLOR_BGR2GRAY);
  
    // Initiate ORB detector
    Ptr<FeatureDetector> detector = ORB::create();
    
    Mat descriptors;
// find the keypoints and descriptors with ORB
    
    detector->detect(src, keypoints);
    Mat im_with_keypoints;
    drawKeypoints( src, keypoints, im_with_keypoints, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );
    
    // Show blobs
    resize(im_with_keypoints, im_with_keypoints, cv::Size(im_with_keypoints.cols/2, im_with_keypoints.rows/2));
    resize(src, src, cv::Size(src.cols/2, src.rows/2));
     
    imshow("keypoints", im_with_keypoints);
    waitKey(1);

 
}
void cb_depth( const sensor_msgs::Image &msg){
    cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(msg);
    imshow("alinhado", depth_image_ptr->image);
    waitKey(1);

}
void cb_scan (const sensor_msgs::LaserScanConstPtr& scan_in)
{

laser_geometry::LaserProjection projector_;
tf::TransformListener listener_;

  if(!listener_.waitForTransform(
        scan_in->header.frame_id,
        "d435_color_optical_frame",
        scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
        ros::Duration(1.0))){
     return;
  }

  sensor_msgs::PointCloud2 cloud;
  projector_.transformLaserScanToPointCloud("d435_color_optical_frame",*scan_in,
          cloud,listener_);
  ros::Time time_st = ros::Time::now ();
  cloud.header.stamp = time_st;
  pub_cloud_lidar.publish(cloud);        

  // Do something with cloud.
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    get_depth_camera_info();
    get_color_camera_info();
    //  ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, cb_pcl);
    pub_cloud_depth = nh.advertise<sensor_msgs::PointCloud2> ("points_depth", 1);
    pub_cloud_lidar = nh.advertise<sensor_msgs::PointCloud2> ("points_lidar", 1);
    ros::Subscriber rgb_sub = nh.subscribe("d435/color/image_raw", 1, cb_rgb);
   // ros::Subscriber depth_sub = nh.subscribe("/d435/depth/image_raw", 1, cb_depth);
    ros::Subscriber aligned_sub = nh.subscribe("/d435/aligned_depth_to_color/image_raw", 1, cb_align);
    ros::Subscriber scan_sub = nh.subscribe("scan", 1, cb_scan);
    std::cout << "o que e que se passa" << std::endl;
    ros::Subscriber pcl_sub = nh.subscribe("landmarks", 1, cb_pcl);
    while (ros::ok())
    {
        ros::spinOnce();
    }

    destroyAllWindows();
    return 0;
}
