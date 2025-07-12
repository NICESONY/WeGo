#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <math.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <image_transport/image_transport.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <opencv2/imgproc/imgproc_c.h>
cv::Mat depthImg;
bool isDepth=false;
cv::Mat rgbImg;
bool isRgb=false;
double fx=604.7211303710938;
double fy=603.2431640625;
int width=640;
int height=480;

std::vector<std::string> objectClasses;
std::vector<std::string> objectFrames;
std::string cameraFrame;

ros::Publisher imgPub;

cv::Mat rgbImage;
cv::Mat depthImage;

//字符串分割函数
std::vector<std::string> split(std::string str,std::string pattern)
{
  std::string::size_type pos;
  std::vector<std::string> result;
  str+=pattern;//扩展字符串以方便操作
  int size=str.size();
 
  for(int i=0; i<size; i++)
  {
    pos=str.find(pattern,i);
    if(pos<size)
    {
      std::string s=str.substr(i,pos-i);
      result.push_back(s);
      i=pos+pattern.size()-1;
    }
  }
  return result;
}

int isInObjects(std::string object){
    for(int i=0;i<objectClasses.size();i++){
        if(!objectClasses[i].compare(object))
            return i;
    }
    return -1;
}

int comp(const void *parama,const void *paramb)
{
    darknet_ros_msgs::BoundingBox *a=(darknet_ros_msgs::BoundingBox *)parama;
    darknet_ros_msgs::BoundingBox *b=(darknet_ros_msgs::BoundingBox *)paramb;
    int aFlag=isInObjects(a->Class);
    int bFlag=isInObjects(b->Class);
    if(aFlag==-1){
        return 1;
    }else if(bFlag==-1){
        return -1;
    }else{
        if(aFlag!=bFlag)
            return aFlag-bFlag;
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
    int count;
    int indexPre;
    for(int i=0;i<end(msg->bounding_boxes)-begin(msg->bounding_boxes);i++){
        int index=isInObjects(bounding_boxes[i].Class);
        if(index==-1){
            std::cout<<"warn: "<<bounding_boxes[i].Class<<" not on the list! "<<std::endl;
            continue;
        }
        if((bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2>depthImg.cols || (bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2>depthImg.rows){
            std::cout<<"error: box out of range! "<<std::endl;
            return;
        }
        std::cout<<"info: "<<bounding_boxes[i].Class<<" in camera:"<<"x:"<<(bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2<<"  y:"<<(bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2<<std::endl;
        
        //计算旋转角度
        Rect area(bounding_boxes[i].xmin,bounding_boxes[i].ymin,bounding_boxes[i].xmax-bounding_boxes[i].xmin,bounding_boxes[i].ymax-bounding_boxes[i].ymin);
        Mat objectRgbImg=rgbImg(area);
        Mat objectRgbImgFilter;
        bilateralFilter(objectRgbImg,objectRgbImgFilter,9,75,75);
        Mat objectGrayImg;

        cvtColor(objectRgbImgFilter, objectGrayImg, CV_BGR2GRAY);
        Canny(objectGrayImg, objectGrayImg,50,100);

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(objectGrayImg,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
        vector<Point> points;
        for(int t=0;t<contours.size();t++){
            for(int j=0;j<contours[t].size();j++){
                points.push_back(contours[t][j]);
            }
        }

        RotatedRect rotRect = minAreaRect(points);
        Mat cornerPoint;
        boxPoints(rotRect, cornerPoint);
        double length1=pow((cornerPoint.at<uchar>(0,0)-cornerPoint.at<uchar>(1,0)),2)+pow((cornerPoint.at<uchar>(0,1)-cornerPoint.at<uchar>(1,1)),2);
        double length2=pow((cornerPoint.at<uchar>(1,0)-cornerPoint.at<uchar>(2,0)),2)+pow((cornerPoint.at<uchar>(1,1)-cornerPoint.at<uchar>(2,1)),2);
        double slope;
        double angle;
        if((length1>=length2&&(cornerPoint.at<uchar>(0,0)-cornerPoint.at<uchar>(1,0)==0))||(length1<=length2&&(cornerPoint.at<uchar>(1,0)-cornerPoint.at<uchar>(2,0)==0)))
            angle=3.1415/2;
        else{
            slope=length1>length2?(-cornerPoint.at<uchar>(0,1)+cornerPoint.at<uchar>(1,1))/(cornerPoint.at<uchar>(0,0)-cornerPoint.at<uchar>(1,0)):(-cornerPoint.at<uchar>(1,1)+cornerPoint.at<uchar>(2,1))/(cornerPoint.at<uchar>(1,0)-cornerPoint.at<uchar>(2,0));
            angle=atan(slope);///3.14*180;
        }   

		
        //初始化坐标计算数据
        double x,y,z,xy;
        int depth;
        //获取深度信息
        depth=depthImg.at<uint16_t>((bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2,(bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2) ;
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


        if(i==0||objectFrames[index].compare(objectFrames[indexPre]))
            count=-1;
        count++;
        indexPre=index;


        //发布坐标
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(z,-x,y) );
        tf::Quaternion q;
        q.setRPY(angle, 0, 0);
        transform.setRotation(q);
        ros::Time rosTime=ros::Time::now();
        std::string frame=string(objectFrames[index]);
        string num;
        stringstream ss;
        ss << count;
        ss >> num;
        frame.append(num);
        br.sendTransform(tf::StampedTransform(transform, rosTime, cameraFrame, frame));

        Rect rect(bounding_boxes[i].xmin,bounding_boxes[i].ymin,bounding_boxes[i].xmax-bounding_boxes[i].xmin,bounding_boxes[i].ymax-bounding_boxes[i].ymin);//左上坐标（x,y）和矩形的长(x)宽(y)

        Scalar scalar(rand()%255,rand()%255,rand()%255);
        cv::putText(rgbImage,objectFrames[index],cv::Point(bounding_boxes[i].xmin,bounding_boxes[i].ymin), cv::FONT_HERSHEY_TRIPLEX, 0.8, scalar, 2, CV_AA);
        cv::rectangle(rgbImage, rect, scalar,1, LINE_8,0);
    }
    sensor_msgs::ImagePtr imageRos = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgbImage).toImageMsg(); 
    imgPub.publish(imageRos);       
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_located");
    ros::NodeHandle node;
    ros::Subscriber depthSub;
    ros::Subscriber rgbSub;
    ros::Subscriber locateSub;
    srand((unsigned)time(NULL));
    std::string objectClass;
    ros::param::get("~object_class",objectClass);
    objectClasses=split(objectClass,"/");
    std::string objectFrame;
    ros::param::get("~object_frame",objectFrame);
    objectFrames=split(objectFrame,"/");
    ros::param::get("~camera_frame",cameraFrame);
    ros::param::get("~fx",fx);
    ros::param::get("~fy",fx);
    depthSub = node.subscribe<sensor_msgs::Image>("/camera/aligned_depth_to_color/image_raw", 10, depthCallback);
    rgbSub = node.subscribe<sensor_msgs::Image>("/camera/color/image_raw", 10, rgbCallback);
    locateSub = node.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 10, locateCallback);
    imgPub=node.advertise<sensor_msgs::Image>("/object_located/image_raw", 10);

    ros::spin();
    return 0;
}

