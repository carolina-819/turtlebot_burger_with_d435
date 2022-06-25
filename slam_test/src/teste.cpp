#include "../include/teste.h"
#include "../include/librealsense2/rsutil.h"
#include "../include/librealsense2/h/rs_types.h"
#include "../include/librealsense2/h/rs_frame.h"
#include <iostream>
#include <fstream>
#include "../include/matplotlibcpp.h"
#include <opencv2/features2d.hpp>
using namespace cv;
using namespace matplotlibcpp;

void show_wait_destroy(const char* winname, Mat img);
void check_distance(std::vector<zone> &zonas, Point ponto, float distancia);
// get camera info
// get pointcloud
// convert pointcloud msg to pointcloud
// iterate through pointcloud to convert points in space to points in image
// create image
// publish image


void cb_pcl(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    depth_map = Mat(cam_info.height, cam_info.width, CV_32FC1, Scalar(std::numeric_limits<float>::max()));

    pcl::PCLPointCloud2 pcl_pc2;

    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *pc);

    for (int i = 0; i < pc->points.size(); i++)
    {
        if (pc->points[i].z == pc->points[i].z && pc->points[i].z >= 0) // NaN values have the odd property that comparisons involving them are always false
        {

            float z = pc->points[i].z;
            float u = (pc->points[i].x * camera_params.fx) / z;
            float v = (pc->points[i].y * camera_params.fy) / z;
            int pixel_pos_x = (int)(u + camera_params.cx);
            int pixel_pos_y = (int)(v + camera_params.cy);
            if (pixel_pos_x > (cam_info.width - 1) || pixel_pos_y > (cam_info.height - 1))
                continue;
            depth_map.at<float>(pixel_pos_y, pixel_pos_x) = z * 10;
        }
    }
    depth_map.convertTo(depth_map, CV_8UC1);

    Mat smaller_img; // copy to vizualize in smaller window, similiar to YOLO window
    int dilation_size = 5;
    resize(depth_map, smaller_img, Size(600, 500));
    Mat element = getStructuringElement(MORPH_RECT,
                                                Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                Point(dilation_size, dilation_size));
    Mat dilation_dst;
    dilate(smaller_img, dilation_dst, element);
    imshow("PCL image", dilation_dst);
    waitKey(1);
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
            intriseco.ppx = cam_info.K[2];
            intriseco.ppy = cam_info.K[5];
            intriseco.fy = cam_info.K[4];
            intriseco.fx = cam_info.K[0];
            intriseco.height = cam_info.height;
            intriseco.width = cam_info.width;
            frame_depth = cam_info.header.frame_id;
            if (cam_info.distortion_model == "plumb_bob") {
                intriseco.model =  RS2_DISTORTION_BROWN_CONRADY;
            }else if  (cam_info.distortion_model == "equidistant") {
                intriseco.model = RS2_DISTORTION_KANNALA_BRANDT4;
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

void get_image_camera_info()
{
    boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo;
    do
    {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/d435/color/camera_info", ros::Duration(5));
        ROS_WARN_STREAM("do");
        if (sharedCameraInfo != NULL)
        {
            cam_info = *sharedCameraInfo;
           
            frame_camera = cam_info.header.frame_id;
            
        }
        else
        {
            ROS_ERROR("Couldn't get image camera info! Trying again...");
            ros::Duration(1.0).sleep();
        }
    } while (sharedCameraInfo == NULL);
    ROS_WARN_STREAM("done"); // esta preso no do
}

/*void check_distance(std::vector<zone> zonas, Point ponto, float distancia){
    float min_diff = 1000; //RESOLVER PROBLEMA DO ENDERENÇO
    int index = 0;
    float tolerancia = 50;
    //find closest zone
    for(int i=0; i<zonas.size(); i++){
        float diff = zonas.at(i).avg - distancia;
        if(diff < min_diff){
            min_diff = diff;
            index = i;
        }
    }
    //if min diff is to big, create new zone
    if(min_diff > tolerancia){
        zone aux;
        aux.avg = distancia;
        aux.pontos.push_back(ponto);
        zonas.push_back(aux);
    }else{ //found a zone, insert in that zone, calculate new avg
        float old_avg = zonas.at(index).avg;
        int old_size = zonas.at(index).pontos.size();
        float new_avg = ((old_avg * old_size) + distancia) / (old_size + 1);
        zonas.at(index).pontos.push_back(ponto);
        zonas.at(index).avg = new_avg;
    }

}*/
void cb_align(const sensor_msgs::ImageConstPtr& msg)
{
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
    std::cout << "min: " << minVal << " max: " << maxVal << std::endl;

    Mat x_coord(intriseco.height, intriseco.width, CV_32F);
    Mat y_coord(intriseco.height, intriseco.width, CV_32F);
    Mat z_coord(intriseco.height, intriseco.width, CV_32F);
    
    for(u_int16_t i = 0; i < h; i++){ //goes through rows
        for (u_int16_t j = 0; j < w; j++){ //goes through columns
            float pixel[2] = {i, j};
            
            float point[3];
            rs2_deproject_pixel_to_point( point, &intriseco, pixel, img.at<u_int16_t>(i, j));
            graymat.at<unsigned char>(i, j) = uchar ((img.at<u_int16_t>(i, j) * 255)/maxVal);
         //   std::cout << "checkpoint " << point[0] << " " << point[1] << " " << point[2] << "depth " << img.at<u_int16_t>(i, j) << std::endl;
         //   std::cout << "valor " << img.at<u_int16_t>(i, j) << std::endl;
            x_coord.at<float>(i, j) =  point[1] * 0.001;
            y_coord.at<float>(i, j) = point[0] * 0.001;
            z_coord.at<float>(i, j) = point[2] * 0.001;
          //  std::cout << "center: " << abs((i-(h/2) * img.at<u_int16_t>(j,i)) / intriseco.fy * 0.01) << "m" << std::endl;
        }
        
    }
    imshow("image grey", graymat);
    waitKey(1);
    Mat im_with_kp;
   // resize(graymat, graymat, cv::Size(graymat.cols/2, graymat.rows/2));
    // Initiate ORB detector
    Ptr<FeatureDetector> detector = ORB::create();
    
    Mat descriptors;
// find the keypoints and descriptors with ORB
    
std::vector<cv::KeyPoint> keypoints_opt;
    detector->detect(graymat, keypoints_opt);
    drawKeypoints( graymat, keypoints, im_with_kp, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );
    
    drawKeypoints( im_with_kp, keypoints_opt, im_with_kp, Scalar(255,255,0), DrawMatchesFlags::DEFAULT );
    imshow("image grey", im_with_kp);
    waitKey(1);
   // ros::Publisher chatter_pub = nh.advertise<std_msgs::Image>("chatter", 1000);
    Mat ex = img.clone();
   
    Mat cut(h, w, CV_8UC1);
    std::vector<float> Z_points;
    std::vector<float> X_points;

    for(int i = 0; i < w; i++){ //goes through columns
        Mat Mi = ex.col(i).clone();
        float min = 100000000;
        int index = 0;
        
            for(int j = 0; j < h; j++){
               // MyFile << img.at<u_int16_t>(j, i) << " ";
               //std::cout << "valores " << img.at<u_int16_t>(j, i)  << " coluna " << Mi.at<float>(j, 0) << std::endl;
                
                if((y_coord.at<float>(j, i) > 0.5 || y_coord.at<float>(j, i) < -2) || img.at<u_int16_t>(j, i) == 0){
                   cut.at<unsigned char>(j, i) = uchar (255);
                }else{
                    cut.at<unsigned char>(j, i) = graymat.at<unsigned char>(j, i);
                    if( (Mi.at<u_int16_t>(j, 0) < min) && (Mi.at<u_int16_t>(j, 0) > 0) ){ 
                    min = Mi.at<u_int16_t>(0, j);
                    index = j;
                }
                }
            }
         //   MyFile << "\t" << "     novo" << "\t";

        //std::cout << "tam: " << Mi.rows << "minimo " <<  Mi.at<float>(index, 0) << "indice " << i << "index " << index << std::endl;
      // nao funciona  int k = std::distance(column.begin() ,  std::min_element(column.begin(), column.end()));
        Z_points.push_back(z_coord.at<float>(index, i)*100);
        X_points.push_back(x_coord.at<float>(index, i));
      //  std::cout << "z points: " << Z_points[i] << std::endl;
    }

    //project into new frame
  //  MyFile << std::endl;
    

    //METER ESTE DIST A FUNCIONAR, FAZER O MAPA DA VISAO DE TOP LEVEL. PASSAR PARA A DETETÇÃO DE OBSTACULOS COM A IMAGEM VISUAL
  //  imshow("image cut", cut);
  //  waitKey(1);

    
    matplotlibcpp::plot(X_points, Z_points);
   // matplotlibcpp::ylim(0, 2);
  //  matplotlibcpp::xlim(-1, 1);
    matplotlibcpp::title("Standard usage"); // set a title
  //  matplotlibcpp::show();
  
/*
auto depth = rs2::get_depth_frame();
auto depth_profile = depth.get_profile().as<rs2::video_stream_profile>();
auto depth_intrin = depth_profile.get_intrinsics();

auto color = fs.get_color_frame();
auto color_profile = color.get_profile().as<rs2::video_stream_profile>();
auto color_intrin = color_profile.get_intrinsics();

auto extrin = depth_profile.get_extrinsics_to(color_profile);
// Maybe color_profile.get_extrinsics_to(depth_profile); ??

float xyz[3];
float color_xyz[3];
float ij[2] = { 100, 100 }; // Pixel indexes
float color_ij[2]; // Matching color pixel

rs2_deproject_pixel_to_point(xyz, &depth_intrin, ij, depth.get_distance(ij[0], ij[1]));
rs2_transform_point_to_point(color_xyz, extrin, xyz);
rs2_project_point_to_pixel(color_ij, color_intrin, color_xyz);
 */ /*  float point[3];
    float local[2] = [0,0];
    rs2_deproject_pixel_to_point(point, &intriseco, local, depth_image_ptr->image.at(0,0))*/

    //processamento de imagem com cor!
/*
    Mat image = depth_image_ptr->image;
    Mat image_canny = depth_image_to_canny->image;
    imshow("depth", image);
    waitKey(1);*/
  /*  Mat edges;
    Canny(image_canny, edges, 0.2, 0.8, 3, false);
    //image.setTo(Scalar(200, 50, 0, 255), edges);

    // Create the images that will use to extract the horizontal and vertical lines
    Mat vertical = edges.clone();
    
    // Specify size on vertical axis
    int vertical_size = vertical.rows / 30;
    // Create structure element for extracting vertical lines through morphology operations
    Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, vertical_size));
    // Apply morphology operations
    erode(vertical, vertical, verticalStructure, Point(-1, -1));
    dilate(vertical, vertical, verticalStructure, Point(-1, -1));
    

    ROS_WARN_STREAM("vertical rows: " << vertical.rows << "ivertical cols " << vertical.cols);
    for(int y = 0; y < vertical.rows; y++) {
        for(int x = 0; x < vertical.cols; x++) {
            float value = vertical.at<float>(y, x);
            if(value > 0){
                float distance = image.at<int>(y, x);
                check_distance(*zones, Point(y, x), distance);
            }
            //ROS_WARN_STREAM("x: " << x << " y: " << y << " e o que ta la: " << value);
        }
    }
   // resize(vertical, vertical, Size(vertical.cols/2, vertical.rows/2));
  //  imshow("vertical", image);
   // waitKey(1);

    srand(time(0)); 
    Mat hora_da_verdade = Mat(image.rows, image.cols, image.type());
    ROS_WARN_STREAM("image rows: " << image.rows << "image cols " << image.cols);
    for(int i = 0; i < zones.size(); i++){
        Vec3b color;
        color[0] = 1 + (rand()) / ( (RAND_MAX/(254-1)));
        color[1] = 1 + (rand()) / ( (RAND_MAX/(254-1)));
        color[2] = 1 + (rand()) / ( (RAND_MAX/(254-1)));
        for(int j = 0; j < zones.at(i).pontos.size(); j++){
            hora_da_verdade.at<Vec3b>(zones.at(i).pontos.at(j).y, zones.at(i).pontos.at(j).x) = color;
        }
    }
    //tentar pintar uma nova imagem com cores diferentes para as diferentes zonas OU por o valor do average no primeiro ponto de cada zona na imagem original :D
    show_wait_destroy("o que e isto", hora_da_verdade);
    //ROS_WARN_STREAM("tem alguma coisa?: " << zones.size());
    
    //passos: -percorrer edges -detetar pontos brancos -ir buscar distancias a depth map original -agrupar por proximidades (fazer funcao) 
    //funcao de agrupar por proximidades: -ver cada ponto branco -percorrer vetor de zonas, ver a qual é que esta mais perto (distancia menor que 0.3 da media) 
    //-se nao houver nenhum criar nova zona
     

    //proximo passo: agrupar as linhas verticais por profundidade para ter noçao dos obstaculos?
   // imshow("imagem de profundidade", im_with_keypoints);
   //resize(image, image, Size(image.cols/2, image.rows/2));
   resize(edges, edges, Size(edges.cols/2, edges.rows/2));
    imshow("image 8 bit", edges);
    waitKey(1);*/
}
void cb_rgb(const sensor_msgs::Image &msg){ //detectar objetos, meter bounding boxes nos objetos
//converter para greyscale, ver se já da keypoints
    cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(msg);
    //imshow("cor", depth_image_ptr->image);
   // waitKey(1);

    Mat src = depth_image_ptr->image.clone();
    Mat greyMat;
    cv::cvtColor(src, greyMat, COLOR_BGR2GRAY);
  //  cv::Mat img_bw;
 //  cv::threshold(greyMat, img_bw, 128.0, 255.0, THRESH_BINARY);
    
   // cv::Mat edges;
  //  cv::Canny(greyMat, edges, 0.7, 0.8, 3, false);
  // resize(img_bw, img_bw, cv::Size(greyMat.cols/2, greyMat.rows/2));
    
  //  imshow("edges", img_bw);

  //  waitKey();
    // Detect blobs.

  /*  SimpleBlobDetector::Params params;
    // Change thresholds
    params.minThreshold = 0;
    params.maxThreshold = 20000;

    // Filter by Area.
    params.filterByArea = false;
   // params.minArea = 1500;

    // Filter by Circularity
    params.filterByCircularity = false;
   // params.minCircularity = 0.1;

    // Filter by Convexity
    params.filterByConvexity = false;
  //  params.minConvexity = 0.87;

    // Filter by Inertia
    params.filterByInertia = false;
  //  params.minInertiaRatio = 0.01;

    // Storage for blobs
    std::vector<KeyPoint> keypoints;

    // Set up detector with params
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
*/
    // Initiate ORB detector
    Ptr<FeatureDetector> detector = ORB::create();
    
    Mat descriptors;
// find the keypoints and descriptors with ORB
    
    detector->detect(src, keypoints);

  //  Ptr<DescriptorExtractor> extractor = ORB::create();
  //  extractor->compute(img_bw, keypoints, descriptors);

    // Detect blobs
   // detector->detect(src, keypoints);
    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    drawKeypoints( src, keypoints, im_with_keypoints, Scalar(255,0,0), DrawMatchesFlags::DEFAULT );
  //  im_with_keypoints.setTo(cv::Scalar(200, 50, 0, 255), keypoints);
    
    // Show blobs
    resize(im_with_keypoints, im_with_keypoints, cv::Size(im_with_keypoints.cols/2, im_with_keypoints.rows/2));
    resize(src, src, cv::Size(src.cols/2, src.rows/2));
  //  imshow("cinzento", greyMat );
  //  waitKey(1);
    
    imshow("keypoints", im_with_keypoints);
    waitKey(1);

 /*   Mat p = Mat::zeros(src.cols*src.rows, 5, CV_32F);
    Mat bestLabels, centers, clustered;
    std::vector<Mat> bgr;
    split(src, bgr);
    // i think there is a better way to split pixel bgr color
    for(int i=0; i<src.cols*src.rows; i++) {
        p.at<float>(i,0) = (i/src.cols) / src.rows;
        p.at<float>(i,1) = (i%src.cols) / src.cols;
        p.at<float>(i,2) = bgr[0].data[i] / 255.0;
        p.at<float>(i,3) = bgr[1].data[i] / 255.0;
        p.at<float>(i,4) = bgr[2].data[i] / 255.0;
    }

    int K = 5;
    kmeans(p, K, bestLabels,
            TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
            3, KMEANS_PP_CENTERS, centers);

    int colors[K];
    for(int i=0; i<K; i++) {
        colors[i] = 255/(i+1);
    }
    // i think there is a better way to do this mayebe some Mat::reshape?
    clustered = Mat(src.rows, src.cols, CV_32F);
    for(int i=0; i<src.cols*src.rows; i++) {
        clustered.at<float>(i/src.cols, i%src.cols) = (float)(colors[bestLabels.at<int>(0,i)]);
//      cout << bestLabels.at<int>(0,i) << " " << 
//              colors[bestLabels.at<int>(0,i)] << " " << 
//              clustered.at<float>(i/src.cols, i%src.cols) << " " <<
//              endl;
    }

    clustered.convertTo(clustered, CV_8U);
    imshow("clustered", clustered);

    waitKey();
*/
}
void cb_depth( const sensor_msgs::Image &msg){
    std::cout << "CARALHO" << std::endl;
    cv_bridge::CvImagePtr depth_image_ptr = cv_bridge::toCvCopy(msg);
    imshow("alinhado", depth_image_ptr->image);
    waitKey(1);

}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;
    get_depth_camera_info();
    get_image_camera_info();
    //  ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, cb_pcl);
    
    ros::Subscriber rgb_sub = nh.subscribe("d435/color/image_raw", 1, cb_rgb);
   // ros::Subscriber depth_sub = nh.subscribe("/d435/depth/image_raw", 1, cb_depth);
    ros::Subscriber aligned_sub = nh.subscribe("/d435/aligned_depth_to_color/image_raw", 1, cb_align);
    
    while (ros::ok())
    {
        ros::spinOnce();
    }

    destroyAllWindows();
    return 0;
}
void show_wait_destroy(const char* winname, Mat img) {
    imshow(winname, img);
    moveWindow(winname, 500, 0);
    waitKey(0);
    destroyWindow(winname);
}
