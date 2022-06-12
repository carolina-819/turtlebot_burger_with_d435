#include "../include/teste.h"
#include "../include/librealsense2/rsutil.h"
#include "../include/librealsense2/h/rs_types.h"
using namespace cv;

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
void get_camera_info()
{
    boost::shared_ptr<sensor_msgs::CameraInfo const> sharedCameraInfo;

    do
    {
        sharedCameraInfo = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/depth/camera_info", ros::Duration(5));
        ROS_WARN_STREAM("do");
        if (sharedCameraInfo != NULL)
        {
            cam_info = *sharedCameraInfo;
            camera_params.cx = cam_info.K[2];
            camera_params.fy = cam_info.K[4];
            camera_params.cy = cam_info.K[5];
            camera_params.fx = cam_info.K[0];

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
void cb_depth(const sensor_msgs::Image &msg)
{
   // std::vector<zone> zones;

    cv_bridge::CvImagePtr depth_image_ptr;
    cv_bridge::CvImagePtr depth_image_to_canny;
    
    try
    {
        depth_image_ptr = cv_bridge::toCvCopy(msg);
        depth_image_to_canny = cv_bridge::toCvCopy(msg, "8UC1");

    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    ROS_WARN_STREAM("TAMANHO " << depth_image_ptr->image.size());
    imshow("image 8 bit", depth_image_ptr->image);
    waitKey(1);

    rs2_intrinsics intriseco;
    intriseco.height = 0;
  /*  float point[3];
    float local[2] = [0,0];
    rs2_deproject_pixel_to_point(point, &intriseco, local, depth_image_ptr->image.at(0,0))*/
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

    Mat src = depth_image_ptr->image;
    Mat greyMat;
    cv::cvtColor(depth_image_ptr->image, greyMat, COLOR_BGR2GRAY);
    cv::Mat img_bw;
    cv::threshold(greyMat, img_bw, 128.0, 255.0, THRESH_BINARY);
    
    cv::Mat edges;
    cv::Canny(greyMat, edges, 0.7, 0.8, 3, false);
    
    // Detect blobs.

    SimpleBlobDetector::Params params;
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

    // Detect blobs
   // detector->detect(src, keypoints);
    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    drawKeypoints( src, keypoints, im_with_keypoints, Scalar(255,0,255), DrawMatchesFlags::DEFAULT );
    im_with_keypoints.setTo(cv::Scalar(200, 50, 0, 255), edges);
    
    // Show blobs
    resize(im_with_keypoints, im_with_keypoints, cv::Size(im_with_keypoints.cols/2, im_with_keypoints.rows/2));
    
    imshow("keypoints", im_with_keypoints );
    waitKey(1);
    resize(img_bw, img_bw, cv::Size(img_bw.cols/2, img_bw.rows/2));
    imshow("binary", img_bw );
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
int main(int argc, char **argv)
{

    ros::init(argc, argv, "test");

    ros::NodeHandle nh;

    //  ros::Subscriber pcl_sub = nh.subscribe("/camera/depth_registered/points", 1, cb_pcl);
    ros::Subscriber depth_sub = nh.subscribe("camera/depth/image", 1, cb_depth);
  //  ros::Subscriber rgb_sub = nh.subscribe("camera/rgb/image_color", 1, cb_rgb);
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
