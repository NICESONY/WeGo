#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <math.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/imgproc/imgproc_c.h>

cv::Mat depthImg;
bool isDepth=false;
cv::Mat rgbImg;
bool isRgb=false;
double fx=604.7211303710938;
double fy=603.2431640625;
int width=640;
int height=480;

ros::Publisher imgPub;

cv::Mat rgbImage;
cv::Mat depthImage;

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
    for(int i=0;i<end(msg->bounding_boxes)-begin(msg->bounding_boxes);i++){
        if(msg->bounding_boxes[i].Class[0]!='w'&&msg->bounding_boxes[i].Class[0]!='p'&&msg->bounding_boxes[i].Class[0]!='i')
            continue;
        if((msg->bounding_boxes[i].xmin+msg->bounding_boxes[i].xmax)/2>depthImage.cols || (msg->bounding_boxes[i].ymin+msg->bounding_boxes[i].ymax)/2>depthImage.rows){
            std::cout<<"depthImage.cols:"<<depthImage.cols<<"  depthImage.rows:"<<depthImage.rows<<std::endl;
            std::cout<<"x:"<<(msg->bounding_boxes[i].xmin+msg->bounding_boxes[i].xmax)/2<<"  y:"<<(msg->bounding_boxes[i].ymin+msg->bounding_boxes[i].ymax)/2<<std::endl;
            return;
        }
        std::cout<<msg->bounding_boxes[i].Class<<" in camera:"<<"x:"<<(msg->bounding_boxes[i].xmin+msg->bounding_boxes[i].xmax)/2<<"  y:"<<(msg->bounding_boxes[i].ymin+msg->bounding_boxes[i].ymax)/2<<std::endl;
        //初始化坐标计算数据
        double x,y,z,xy;
        int depth;
        //获取深度信息
        depth=depthImage.at<uint16_t>((msg->bounding_boxes[i].ymin+msg->bounding_boxes[i].ymax)/2,(msg->bounding_boxes[i].xmin+msg->bounding_boxes[i].xmax)/2) ;
        //计算中心点
        x=1.0*(msg->bounding_boxes[i].xmin+msg->bounding_boxes[i].xmax)/2-1.0*width/2;
        y=1.0*height/2-(msg->bounding_boxes[i].ymin+msg->bounding_boxes[i].ymax)/2;
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
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera_color_frame", msg->bounding_boxes[i].Class));
        Rect rect(msg->bounding_boxes[i].xmin,msg->bounding_boxes[i].ymin,msg->bounding_boxes[i].xmax-msg->bounding_boxes[i].xmin,msg->bounding_boxes[i].ymax-msg->bounding_boxes[i].ymin);//左上坐标（x,y）和矩形的长(x)宽(y)
        Scalar scalar(rand()%255,rand()%255,rand()%255);
        cv::putText(rgbImage,msg->bounding_boxes[i].Class,cv::Point(msg->bounding_boxes[i].xmin,msg->bounding_boxes[i].ymin), cv::FONT_HERSHEY_TRIPLEX, 0.8, scalar, 2, CV_AA);
        cv::rectangle(rgbImage, rect, scalar,1, LINE_8,0);
    }
    sensor_msgs::ImagePtr imageRos = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImage).toImageMsg(); 
    imgPub.publish(imageRos);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "traffic_sign_located");
    ros::NodeHandle node;
    ros::Subscriber depthSub;
    ros::Subscriber locateSub;
    srand((unsigned)time(NULL));
    ros::param::get("~fx",fx);
    ros::param::get("~fy",fx);
    depthSub = node.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 10, depthCallback);
    locateSub = node.subscribe<darknet_ros_msgs::BoundingBoxes>("/traffic_sign/darknet_ros/bounding_boxes", 10, locateCallback);
    imgPub=node.advertise<sensor_msgs::Image>("/traffic_sign_located/image_raw", 10);
    ros::spin();
    return 0;
}
