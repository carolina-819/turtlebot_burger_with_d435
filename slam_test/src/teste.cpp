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

std::vector<double> sensor_fusion(double x_old, double y_old, double x_meas, double y_meas, double relation, double range, double teta){
    // range e teta são os valores que vem do lidar
    // old sao os valores da camara, meas sao os valores do lidar que corrigem
    float old_x_variance, old_y_variance, meas_x_variance, meas_y_variance;
    // camara, o x é a distancia, que vai ser o z, que tem de incerteza os 2% certos
    old_x_variance = std::pow(sigma_vision, 2);
    old_y_variance = std::pow(sigma_vision*(relation), 2);

    // lidar, a variancia está associada ao range
    // TODO receber o range e o teta. o range serve p saber qual sigma usar
    if (range * 1000  <= 499){
        meas_x_variance = std::pow(sigma_lidar1 * std::cos(teta) * range * 1000, 2);
        meas_y_variance = std::pow(sigma_lidar1 * std::sin(teta) * range * 1000, 2);
    }else{
        meas_x_variance = std::pow(sigma_lidar2 * std::cos(teta) * range * 1000, 2);
        meas_y_variance = std::pow(sigma_lidar2 * std::sin(teta) * range * 1000, 2);
    }
    
    // CALCULATE X and y
    
    double x = (old_x_variance*x_old + x_meas*meas_x_variance)/(old_x_variance+meas_x_variance);
    double y = (old_y_variance*y_old + y_meas*meas_y_variance)/(old_y_variance+meas_y_variance);
    return std::vector<double> {x, y};
}

void cb_pcl(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PCLPointCloud2 pcl_toda;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_toda(new pcl::PointCloud<pcl::PointXYZ>);
    if(msg){
        pcl_conversions::toPCL(*msg, pcl_toda);
        pcl::fromPCLPointCloud2(pcl_toda, *pc_toda);

       // std::cout << " tamanho recebido " << pc_toda->points.size() << std::endl;
   
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

void cb_align(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::LaserScanConstPtr& lidar_msg) 
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
            graymat.at<unsigned char>(i, j) = uchar ((img.at<u_int16_t>(i, j) * 255)/maxVal);

            x_coord.at<float>(i, j) = ((j - intriseco.ppx)/(intriseco.fx)) * img.at<u_int16_t>(i, j) * 0.001;
            y_coord.at<float>(i, j) = ((i - intriseco.ppy)/(intriseco.fy)) * img.at<u_int16_t>(i, j) * 0.001;
            z_coord.at<float>(i, j) = img.at<u_int16_t>(i, j) * 0.001;
            
        }
    }
    
  //  Mat im_with_kp;
   // cv::drawKeypoints( graymat, keypoints, im_with_kp, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );
    
 //   cv::imshow("image grey", im_with_kp);
 //   cv::waitKey(1);

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud->header.frame_id = "d435_color_optical_frame";
    
    int size_pcl = 0;
    for (int i = 0; i < keypoints_new.size(); i++){
        // TODO FILTRAR KEYPOINTS AQUI COM A CENA DO LIDAR
        pcl::PointXYZ ponto;
        ponto.z = z_coord.at<float>(keypoints_new[i].pt.y, keypoints_new[i].pt.x);
        ponto.y = y_coord.at<float>(keypoints_new[i].pt.y, keypoints_new[i].pt.x);
        ponto.x = x_coord.at<float>(keypoints_new[i].pt.y, keypoints_new[i].pt.x);
    

        //filtra pontos
         // passa para a frame do lidar, valores da camara
         double x_cam = ponto.z - dx;
         double y_cam = dy - ponto.x;
         double bearing_cam = std::atan2(y_cam, x_cam); // bearing aproximado
        double bearing_lidar = 0;
        int pos = bearing_cam/(lidar_msg->angle_increment);
        double distance = -1;
        int aux = -1;
        // otimizar para ter o range cuja diferença é menor
        if((abs(lidar_msg->ranges[pos] - ( std::sqrt(std::pow(x_cam, 2) + std::pow(y_cam, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos] > 0)) ) {
            aux = pos;
        }
        else if((abs(lidar_msg->ranges[pos + 1] - ( std::sqrt(std::pow(x_cam, 2) + std::pow(y_cam, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos + 1] > 0)) ) {
           aux = pos + 1;
        }else if((abs(lidar_msg->ranges[pos -1] - ( std::sqrt(std::pow(x_cam, 2) + std::pow(y_cam, 2)) )) < tole ) && (abs(lidar_msg->ranges[pos - 1] > 0)) ) {
            aux = pos - 1;
        }
        if(aux > -1) {
            distance = lidar_msg->ranges[aux];
            bearing_lidar = aux * lidar_msg->angle_increment;
        }
        
        if(distance > 0 || (lidar_msg->ranges[aux] != lidar_msg->ranges[aux])){
           // std::cout << "encontrou correspondencia" << i << std::endl;
            pcl::PointXYZ p;
            if(!lidar_msg->ranges[aux]){
                std::cout << "e um menino (a distancia e infinita)" << lidar_msg->ranges[aux] << std::endl;
                pointcloud->points.push_back(ponto); //nao da para fazer fusao
                
            }else{
                 // passa de volta para as coordenadas da frame de visao, desta vez com a informaçao do lidar, que a partida é mais reliable??
                // fusao de sensores
                // x_cam e y_cam: coordenadas calculadas com os valores que vem da camara, na frame do lidar
                // x_lidar e y_lidar: coordenadas com os valores que vem do lidar, na frame do lidar
                double x_lidar = (distance * std::cos(bearing_lidar));
                double y_lidar = (distance * std::sin(bearing_lidar));
                std::vector<double> new_val;
                new_val = sensor_fusion(x_cam, y_cam, x_lidar, y_lidar, (ponto.x/ponto.z), distance, bearing_lidar);
                p.y = ponto.y;
                p.z = new_val[0] + dx; 
                p.x = dy - new_val[1];
                pointcloud->points.push_back(p);
                std::cout << "x novo " << new_val[0] << " y novo " << new_val[1] << " x_cam " << x_cam << " y_cam " << y_cam << " x_lidar " << x_lidar << " y_lidar " << y_lidar << std::endl;
            }
            size_pcl++;
        }
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
   
    img_new = depth_image_ptr->image.clone();
    Mat greyMat;
    cv::cvtColor(img_new, greyMat, COLOR_BGR2GRAY);
  
    // Initiate ORB detector
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );


    detector->detect ( img_new ,keypoints_new );

    descriptor->compute ( img_new, keypoints_new, descriptors_new);

   if(init){
    init = false;
   }else if(keypoints_old.size() > 0 && keypoints_new.size() > 0){

    std::vector<DMatch> matches;

    matcher->match ( descriptors_old, descriptors_new, matches );

    // find the keypoints and descriptors with ORB
    // TODO filtrar pontos pelo quao distinguishable sao
    //tendo a covariancia dos dois sensores no ponto especifico, calculo o que e que seria o ponto real tendo em conta essas duas covariancia (formulas do filtro de kalman, estado e observaçao xy)
    detector->detect(img_new, keypoints_new);
    Mat img_match;
    drawMatches ( img_old, keypoints_old, img_new, keypoints_new, matches, img_match );
    resize(img_match, img_match, cv::Size(img_match.cols/2, img_match.rows/2));
  //  imshow ( "所有匹配点对", img_match );
  //  waitKey(1);
   }
    //TODO GET DESCRIPTORS
  //  Mat im_with_keypoints;
  //  drawKeypoints( img_new, keypoints_keypoints, im_with_keypoints, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );
    
    // do matching if not init
    
    // update new and old image
    img_old = img_new.clone();
    keypoints_old = keypoints_new;
    descriptors_old = descriptors_new;

  //  resize(im_with_keypoints, im_with_keypoints, cv::Size(im_with_keypoints.cols/2, im_with_keypoints.rows/2));
  //  resize(img_new, img_new, cv::Size(img_new.cols/2, img_new.rows/2));
     
  //  imshow("keypoints", im_with_keypoints);
  //  waitKey(1);

 
}
void cb_depth( const sensor_msgs::Image &msg){
    cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(msg);
 //   imshow("alinhado", depth_image_ptr->image);
 //   waitKey(1);

}

int main(int argc, char **argv)
{
    init = true;
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    get_depth_camera_info();
    get_color_camera_info();
    //  ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, cb_pcl);
    pub_cloud_depth = nh.advertise<sensor_msgs::PointCloud2> ("points_depth", 1);

    ros::Subscriber rgb_sub = nh.subscribe("d435/color/image_raw", 1, cb_rgb);

    message_filters::Subscriber<sensor_msgs::Image> cloud_depth_sub(nh, "/d435/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> lidar_sub(nh, "scan", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> MySyncPolicy;
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cloud_depth_sub, lidar_sub);

    sync.registerCallback(boost::bind(&cb_align, _1, _2));
  
    std::cout << "o que e que se passa" << std::endl;
    while (ros::ok())
    {
        ros::spinOnce();
    }

    destroyAllWindows();
    return 0;
}
