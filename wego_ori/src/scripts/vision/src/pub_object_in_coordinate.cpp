
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

ros::Publisher pub_object_in_base_link;
ros::Publisher pub_object_in_camera;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pub_object_in_coordinate");
    ros::NodeHandle node;
    pub_object_in_base_link = node.advertise<geometry_msgs::PoseStamped>("/object_in_base_link", 10);
    pub_object_in_camera = node.advertise<geometry_msgs::PoseStamped>("/object_in_camera", 10);
    while (node.ok()){
        tf::TransformListener listener_camera;
        tf::StampedTransform listenTransform_camera;
        try{
          listener_camera.waitForTransform("/camera_link", "/apple", ros::Time(0), ros::Duration(2.0));
          listener_camera.lookupTransform("/camera_link", "/apple", ros::Time(0), listenTransform_camera);
        }
        catch (tf::TransformException &ex) {
          //ros::Duration(1.0).sleep();
          continue;
        }
        geometry_msgs::PoseStamped pose_camera;
        pose_camera.header.frame_id = "apple";
        pose_camera.pose.position.x=listenTransform_camera.getOrigin().x();
        pose_camera.pose.position.y=listenTransform_camera.getOrigin().y();
        pose_camera.pose.position.z=listenTransform_camera.getOrigin().z();
        pose_camera.pose.orientation.x=listenTransform_camera.getRotation().x();
        pose_camera.pose.orientation.y=listenTransform_camera.getRotation().y();
        pose_camera.pose.orientation.z=listenTransform_camera.getRotation().z();
        pose_camera.pose.orientation.w=listenTransform_camera.getRotation().w();
        pub_object_in_camera.publish(pose_camera);

        tf::TransformListener listener_base_link;
        tf::StampedTransform listenTransform_base_link;
        try{
          listener_base_link.waitForTransform("/base_link", "/apple",  ros::Time(0), ros::Duration(2.0));
          listener_base_link.lookupTransform("/base_link", "/apple", ros::Time(0), listenTransform_base_link);
        }
        catch (tf::TransformException &ex) {
          //ros::Duration(1.0).sleep();
          continue;
        }
        geometry_msgs::PoseStamped pose_base_link;
        pose_base_link.header.frame_id = "apple";
        pose_base_link.pose.position.x=listenTransform_base_link.getOrigin().x();
        pose_base_link.pose.position.y=listenTransform_base_link.getOrigin().y();
        pose_base_link.pose.position.z=listenTransform_base_link.getOrigin().z();
        pose_base_link.pose.orientation.x=listenTransform_base_link.getRotation().x();
        pose_base_link.pose.orientation.y=listenTransform_base_link.getRotation().y();
        pose_base_link.pose.orientation.z=listenTransform_base_link.getRotation().z();
        pose_base_link.pose.orientation.w=listenTransform_base_link.getRotation().w();
        pub_object_in_base_link.publish(pose_base_link);

    }
    ros::spin();
    return 0;
}

