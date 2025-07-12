#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>
#include <math.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc_c.h>

cv::Mat depthImg;
bool isDepth=false;
cv::Mat rgbImg;
cv::Mat cor_rgbImg;
bool isRgb=false;
double fx=604.7211303710938;
double fy=603.2431640625;
int width=640;
int height=480;

ros::Publisher imgPub;

cv::Mat rgbImage;
cv::Mat depthImage;

int comp(const void *parama,const void *paramb)
{
    darknet_ros_msgs::BoundingBox *a=(darknet_ros_msgs::BoundingBox *)parama;
    darknet_ros_msgs::BoundingBox *b=(darknet_ros_msgs::BoundingBox *)paramb;
    if(a->Class.compare("traffic light")){
        return 1;
    }else if(b->Class.compare("traffic light")){
        return -1;
    }else{
        return depthImage.at<uint16_t>((a->ymin+a->ymax)/2,(a->xmin+a->xmax)/2)-depthImage.at<uint16_t>((b->ymin+b->ymax)/2,(b->xmin+b->xmax)/2);
    }
}

void depthCallback(const sensor_msgs::ImageConstPtr& paramDepthImg) {
    depthImg =cv_bridge::toCvCopy(paramDepthImg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    isDepth=true;
}

void rgbCallback(const sensor_msgs::ImageConstPtr& paramRgbImg){
    rgbImg =cv_bridge::toCvCopy(paramRgbImg, sensor_msgs::image_encodings::BGR8)->image;
    width=rgbImg.cols;
    height=rgbImg.rows;
    isRgb=true;
}

//cv::Mat correctBrightess(cv::Mat src){
    // 调整亮度和对比度
//    double alpha = 1.2;  // 用于调整对比度
//    double beta = 30;  // 用于增加亮度

    // 新建目标图像
 //   cv::Mat dst = Mat::zeros(src.size(), src.type());

  //  int rows = src.rows;
   // int cols = src.cols;

    //for (int row = 0; row < rows - 1; row++){
      //  for (int col = 0; col < cols - 1; col++){
        //    // 获取像素
          //  int b = src.at<Vec3b>(row, col)[0];
            //int g = src.at<Vec3b>(row, col)[1];
           // int r = src.at<Vec3b>(row, col)[2];
            
            // 调整亮度和对比度
            //dst.at<Vec3b>(row.col)[0] = saturate_cast<uchar>(alpha*b+beta);
            //dst.at<Vec3b>(row, col)[0] = saturate_cast<uchar>((alpha*b + beta));
           // dst.at<Vec3b>(row, col)[1] = saturate_cast<uchar>((alpha*g + beta));
            //dst.at<Vec3b>(row, col)[2] = saturate_cast<uchar>((alpha*r + beta));
       // }
   // }
//    return dst;
//}

void locateCallback(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg){

    using namespace std;
    using namespace cv;
  /*cout<<"Bouding Boxes (header):" << msg->header <<endl;
    cout<<"Bouding Boxes (image_header):" << msg->image_header <<endl;
    cout<<"Bouding Boxes (Class):" << msg->bounding_boxes[i].Class <<endl;
    cout<<"Bouding Boxes (xmin):" << msg->bounding_boxes[i].xmin <<endl;
    cout<<"Bouding Boxes (xmax):" << msg->bounding_boxes[i].xmax <<endl;
    cout<<"Bouding Boxes (ymin):" << msg->bounding_boxes[i].ymin <<endl;
    cout<<"Bouding Boxes (ymax):" << msg->bounding_boxes[i].ymax <<endl;
    cout << "\033[2J\033[1;1H";    */ // clear terminal
    if(!isRgb || !isDepth )
        return;
    rgbImage=rgbImg;
    depthImage=depthImg;
    darknet_ros_msgs::BoundingBox *bounding_boxes=new darknet_ros_msgs::BoundingBox[end(msg->bounding_boxes)-begin(msg->bounding_boxes)];
    memcpy(bounding_boxes,&(msg->bounding_boxes[0]),(end(msg->bounding_boxes)-begin(msg->bounding_boxes))*sizeof(darknet_ros_msgs::BoundingBox));
    qsort(bounding_boxes,end(msg->bounding_boxes)-begin(msg->bounding_boxes),sizeof(darknet_ros_msgs::BoundingBox),comp);//调用qsort排序
    for(int i=0;i<end(msg->bounding_boxes)-begin(msg->bounding_boxes);i++){
        if(bounding_boxes[i].Class.compare("traffic light"))
            continue;
        if(!isRgb || !isDepth || (bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2>depthImage.cols || (bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2>depthImage.rows){
            std::cout<<"depthImage.cols:"<<depthImage.cols<<"  depthImage.rows:"<<depthImage.rows<<std::endl;
            std::cout<<"x:"<<(bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2<<"  y:"<<(bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2<<std::endl;
            continue;
        }
        std::cout<<bounding_boxes[i].Class<<" in camera:"<<"x:"<<(bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2<<"  y:"<<(bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2<<std::endl;
        //截取红绿灯区域
        Rect area(bounding_boxes[i].xmin,bounding_boxes[i].ymin,bounding_boxes[i].xmax-bounding_boxes[i].xmin,bounding_boxes[i].ymax-bounding_boxes[i].ymin);
        Mat trafficLightRgbImg=rgbImg(area);
        //转换为HSV颜色空间
        Mat trafficLightHsvImg;
        cvtColor(trafficLightRgbImg, trafficLightHsvImg, CV_BGR2HSV);
        //设置阈值
        Mat redGrayImg;
        Mat greenGrayImg;
        Mat yellowGrayImg;

        inRange(trafficLightHsvImg, Scalar(156,43,46), Scalar(180,255,255), redGrayImg);
        inRange(trafficLightHsvImg, Scalar(49,79,137), Scalar(90,255,255), greenGrayImg);
        inRange(trafficLightHsvImg, Scalar(26,43,46), Scalar(34,255,255), yellowGrayImg);

	/*	cv::medianBlur(redGrayImg,redGrayImg,7);
        cv::medianBlur(greenGrayImg,greenGrayImg,7);
        cv::medianBlur(yellowGrayImg,yellowGrayImg,7);*/

        //统计数量进行比较
        int redCount,greenCount,yellowCount;
        redCount=countNonZero(redGrayImg);
        greenCount=countNonZero(greenGrayImg);
        yellowCount=countNonZero(yellowGrayImg);
        cout<<"red:"<<redCount<<" green:"<<greenCount<<" yellow:"<<yellowCount<<endl;
        //结果
        string result;
        if(redCount>=greenCount&&redCount>=yellowCount)
            result="red";
        else if(greenCount>=redCount&&greenCount>=yellowCount)
            result="green";
        else
            result="yellow";
        cout<<result<<endl;

        //初始化坐标计算数据
        double x,y,z,xy;
        int depth;
        //获取深度信息
        depth=depthImage.at<uint16_t>((bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2,(bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2) ;
        //计算中心点
        x=1.0*(bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2-1.0*width/2;
        y=1.0*height/2-(bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2;
        //计算坐标轴
        z=1.0*depth;
        x=1.0*x/fx*z;
        y=1.0*y/fy*z;
        x=0.001*x;
        y=0.001*y;
        z=0.001*z;
        //发布坐标
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(z,-x,y) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_frame", bounding_boxes[i].Class+" "+result));
        Rect rect(bounding_boxes[i].xmin,bounding_boxes[i].ymin,bounding_boxes[i].xmax-bounding_boxes[i].xmin,bounding_boxes[i].ymax-bounding_boxes[i].ymin);//左上坐标（x,y）和矩形的长(x)宽(y)
        Scalar scalar(rand()%255,rand()%255,rand()%255);
        cv::putText(rgbImage,bounding_boxes[i].Class+" "+result,cv::Point(bounding_boxes[i].xmin,bounding_boxes[i].ymin), cv::FONT_HERSHEY_TRIPLEX, 0.8, scalar, 2, CV_AA);
        cv::rectangle(rgbImage, rect, scalar,1, LINE_8,0);
    }
    sensor_msgs::ImagePtr imageRos = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImage).toImageMsg(); 
    imgPub.publish(imageRos);       
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traffic_light_located");
    ros::NodeHandle node;
    ros::Subscriber depthSub;
    ros::Subscriber rgbSub;
    ros::Subscriber locateSub;
    srand((unsigned)time(NULL));
    ros::param::get("~fx",fx);
    ros::param::get("~fy",fx);
    depthSub = node.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 10, depthCallback);
    rgbSub = node.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, rgbCallback);
    locateSub = node.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 10, locateCallback);
    imgPub=node.advertise<sensor_msgs::Image>("/traffic_light_located/image_raw", 10);

    ros::spin();
    return 0;
}

