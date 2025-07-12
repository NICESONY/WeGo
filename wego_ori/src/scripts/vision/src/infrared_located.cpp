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

cv::Mat infImg;
bool isInf=false;

double fx=604.7211303710938;
double fy=603.2431640625;
int width=640;
int height=480;
double camHeight=1.0;//摄像头安装高度
double camCenter=0.0;//摄像头距离中间位置

std::vector<std::string> objectClasses;
std::vector<std::string> objectFrames;
std::string carFrame;

ros::Publisher imgPub;

cv::Mat infImage;


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

void getDistance(int bottomX,int bottomY,double *xy){
    //计算点距离中心
    double x=1.0*bottomX-1.0*width/2;
    double y=1.0*height/2-1.0*bottomY;
    if(y>0){
        xy[0]=-1;
        xy[1]=-1;
        return;
    }
    y=abs(y);
    //计算坐标轴
    double s=sqrt(x*x+fx*fx);
    double distance=s*camHeight/y;
    double vertical=fx*distance/s;
    xy[0]=vertical;
    xy[1]=sqrt(distance*distance-vertical*vertical);
    if(x<0){
        xy[1]=-1.0*xy[1];
    }
    return;
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
        double aXY[2];
        double bXY[2];
        getDistance((a->xmin+a->xmax)/2,a->ymin,aXY);
        getDistance((b->xmin+b->xmax)/2,b->ymin,bXY);
        double result=aXY[0]-bXY[0];
        return result>0?ceil(result):floor(result);
    }
}

void infCallback(const sensor_msgs::ImageConstPtr& paramInfImg) {
    infImg =cv_bridge::toCvCopy(paramInfImg, sensor_msgs::image_encodings::TYPE_8UC1)->image;
    width=infImg.cols;
    height=infImg.rows;
    isInf=true;
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
    if(!isInf)
        return;
    infImage=infImg;
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
        if((bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2>width || (bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2>height){
            std::cout<<"error: box out of range! "<<std::endl;
            return;
        }
        std::cout<<"info: "<<bounding_boxes[i].Class<<" in camera:"<<"x:"<<(bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2<<"  y:"<<(bounding_boxes[i].ymin+bounding_boxes[i].ymax)/2<<std::endl;
        
		
        double xy[2];
        getDistance((bounding_boxes[i].xmin+bounding_boxes[i].xmax)/2,bounding_boxes[i].ymax,xy);
        if(xy[0]==-1){
            std::cout<<"warn: "<<bounding_boxes[i].Class<<" can not reach, camera may not horizontal! "<<std::endl;
            continue;
        }
        if(i==0||objectFrames[index].compare(objectFrames[indexPre]))
            count=-1;
        count++;
        indexPre=index;
        //发布坐标
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(xy[0],xy[1],0));
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        ros::Time rosTime=ros::Time::now();
        std::string frame=string(objectFrames[index]);
        string num;
        stringstream ss;
        ss << count;
        ss >> num;
        frame.append(num);
        //br.sendTransform(tf::StampedTransform(transform, rosTime, carFrame, frame));
        /*Rect rect(bounding_boxes[i].xmin,bounding_boxes[i].ymin,bounding_boxes[i].xmax-bounding_boxes[i].xmin,bounding_boxes[i].ymax-bounding_boxes[i].ymin);//左上坐标（x,y）和矩形的长(x)宽(y)

        Scalar scalar(rand()%255,rand()%255,rand()%255);
        cv::putText(rgbImage,objectFrames[index],cv::Point(bounding_boxes[i].xmin,bounding_boxes[i].ymin), cv::FONT_HERSHEY_TRIPLEX, 0.8, scalar, 2, CV_AA);
        cv::rectangle(rgbImage, rect, scalar,1, LINE_8,0);*/
    }
   /* sensor_msgs::ImagePtr imageRos = cv_bridge::CvImage(std_msgs::Header(), "bgr8", infImage).toImageMsg(); 
    imgPub.publish(imageRos);       */
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "infrared_located");
    ros::NodeHandle node;
    ros::Subscriber infSub;
    ros::Subscriber locateSub;
    srand((unsigned)time(NULL));
    std::string objectClass;
    ros::param::get("~object_class",objectClass);
    objectClasses=split(objectClass,"/");
    std::string objectFrame;
    ros::param::get("~object_frame",objectFrame);
    objectFrames=split(objectFrame,"/");
    ros::param::get("~car_frame",carFrame);
    ros::param::get("~fx",fx);
    ros::param::get("~fy",fx);
    infSub = node.subscribe<sensor_msgs::Image>("/infrared_camera/image_raw", 10, infCallback);
    locateSub = node.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 10, locateCallback);
    imgPub=node.advertise<sensor_msgs::Image>("/infrared_located/image_raw", 10);

    ros::spin();
    return 0;
}

