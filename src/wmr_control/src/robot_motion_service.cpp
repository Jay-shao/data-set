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
#include <random>
#include "wmr_control/robot_move.h"
using namespace std;

#define STOP 0
#define RANDOM_VELOCITY 1
#define ROTATION 2

class robotMotion
{
private:
    ros::NodeHandle nh;
    ros::Publisher robotMovePub_;
    ros::ServiceServer motionServer_;

public:
    robotMotion();
    bool doCmd(wmr_control::robot_moveRequest& req, wmr_control::robot_moveResponse& resp);
    void robotStop(geometry_msgs::Twist& msg, wmr_control::robot_moveRequest& requst);
    void robotRotation(geometry_msgs::Twist& msg, wmr_control::robot_moveRequest& requst);
    void robotVelocity(geometry_msgs::Twist& msg, wmr_control::robot_moveRequest& requst);
    double generateRandomNumber(double minRange, double maxRange);

};

robotMotion::robotMotion()
{
    robotMovePub_ = nh.advertise<geometry_msgs::Twist>("/ranger/cmd_vel", 10, true);
    motionServer_ = nh.advertiseService("robot_motion", &robotMotion::doCmd, this);
}

bool robotMotion::doCmd(wmr_control::robot_moveRequest& req, wmr_control::robot_moveResponse& resp)
{
    geometry_msgs::Twist cmd;
    int the_case = req.motionModeID;
    switch (the_case)
    {
    case 1:
        robotStop(cmd, req);
        resp.flag = true;
        break;

    case 2:
        robotVelocity(cmd, req);
        resp.flag = true;
        break;

    case 3:
        //ROS_WARN("case 3 is in!");
        robotRotation(cmd, req);
        resp.flag = true;
        break;
    
    default:
        ROS_ERROR("robot motion mode ID is Wrrong!!!!");
        ROS_INFO("Mode ID is :%d",the_case);
        
    }
    robotMovePub_.publish(cmd);
    //ROS_WARN("robot velocity is publisher......");
    return true;
}

void robotMotion::robotStop(geometry_msgs::Twist& msg, wmr_control::robot_moveRequest& requst)
{
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
}

void robotMotion::robotVelocity(geometry_msgs::Twist& msg, wmr_control::robot_moveRequest& requst)
{
    msg.linear.x = requst.linear_x;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
}

void robotMotion::robotRotation(geometry_msgs::Twist& msg, wmr_control::robot_moveRequest& requst)
{
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = generateRandomNumber(-1.1, 1.1);
}

double robotMotion::generateRandomNumber(double minRange, double maxRange) {
    // 使用随机设备作为随机数引擎种子
    std::random_device rd;
    
    // 创建一个 Mersenne Twister 随机数引擎
    std::mt19937 gen(rd());
    
    // 创建一个均匀分布的随机数分布，范围在[minRange, maxRange]之间
    std::uniform_real_distribution<double> dis(minRange, maxRange);
    
    // 生成随机数并保留两位小数
    double randomNum = dis(gen);
    randomNum = round(randomNum * 100.0) / 100.0; // 精确到小数点后两位
    
    return randomNum;
} 

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_motion_service");
    robotMotion robot;
    ros::spin();
    return 0;
}







