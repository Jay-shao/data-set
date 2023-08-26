#include "ros/ros.h"
#include <ros/time.h>
#include "std_msgs/Float64.h"
#include <algorithm>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include <iostream>
#include <sys/stat.h>
#include <sys/types.h>
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include "wmr_control/cmd.h"

using namespace std;

double robotAngleOfYaw;


void callback(const nav_msgs::Odometry& odom)
{
    tf::Quaternion quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    robotAngleOfYaw = yaw;
    geometry_msgs::Twist msg;
    double robotPosition_x = odom.pose.pose.position.x;
    ROS_INFO("robotPosition_x is :%.4f |||||| %.4f",robotPosition_x,odom.pose.pose.position.x);
    double robotPosition_y = odom.pose.pose.position.y;
    ROS_INFO("robotPosition_y is :%.4f |||||| %.4f",robotPosition_y,odom.pose.pose.position.y);
    double robotPosition_z = odom.pose.pose.position.z;
    ROS_INFO("robotPosition_z is :%.4f |||||| %.4f",robotPosition_z,odom.pose.pose.position.z);
    
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_test01");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/odom",10,callback);
    ros::ServiceClient client = nh.serviceClient<wmr_control::cmd>("cmd");
    ros::service::waitForService("cmd");

    wmr_control::cmd resp;
    resp.request.linearx = 1.0;
    resp.request.lineary = 0.0;
    resp.request.linearz = 0.0;
    resp.request.angularx = 0.0;
    resp.request.angulary = 0.0;
    resp.request.angularz = 1.0;
    resp.request.angle = robotAngleOfYaw;

    bool flag = client.call(resp);
    if (flag)
    {
        ROS_INFO("service is success!!");
    }
    else{
        ROS_ERROR("service is failed!!");
    }
    
    ros::spin();

}
