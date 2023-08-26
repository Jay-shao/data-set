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
#include "wmr_control/cmd.h"

using namespace std;
class CMD
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::ServiceServer server;


public:
    CMD();
    bool doCmd(wmr_control::cmdRequest& req, wmr_control::cmdResponse& resp);
    void test(geometry_msgs::Twist& msg, wmr_control::cmdRequest& req);
    

};

CMD::CMD()
{
    pub = nh.advertise<geometry_msgs::Twist>("/ranger/cmd_vel",10,true);
    server = nh.advertiseService("cmd",&CMD::doCmd,this);
}


bool CMD::doCmd(wmr_control::cmdRequest& req,
           wmr_control::cmdResponse& resp)
{
    geometry_msgs::Twist msg;
    // msg.linear.x = req.linearx;
    // msg.linear.y = req.lineary;
    // msg.linear.z = req.linearz;
    // msg.angular.x = req.angularx;
    // msg.angular.y = req.angulary;
    // msg.angular.z = req.angularz;
    test(msg, req);
    if(req.linearx == 0 && req.lineary == 0)
    {
        ROS_ERROR("velocity is wrrory");
        resp.flag = false;
        return false;
    }
    else
    {
        pub.publish(msg);
        ROS_WARN("velocity is published");
        ROS_INFO("yaw angle i: %.4f",req.angle);
        resp.flag = true;
        return true;
    }
}

void CMD::test(geometry_msgs::Twist& msg, wmr_control::cmdRequest& req)
{
    msg.linear.x = req.linearx;
    msg.linear.y = req.lineary;
    msg.linear.z = req.linearz;
    msg.angular.x = req.angularx;
    msg.angular.y = req.angulary;
    msg.angular.z = req.angularz;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cmd_test");
    
    CMD cmd;
    ros::spin();

    return 0;
}
