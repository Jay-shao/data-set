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
#include "wmr_control/robot_move.h"
#include <random>

#define STOP 1
#define VELOCITY 2
#define ROTATION 3
using namespace std;

double robotAngleOfYaw;

class RobotMotionTest
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::ServiceClient client;
    wmr_control::robot_move req;
    
public:
    RobotMotionTest();
    void callback(const nav_msgs::Odometry& odom);
    void test();
    bool robotMotionServer(int modeID);
    double generateRandomNumber(double minRange, double maxRange);
    void RandomDelay();

};

RobotMotionTest::RobotMotionTest()
{
    sub = nh.subscribe("/odom",10, &RobotMotionTest::callback, this);
    client = nh.serviceClient<wmr_control::robot_move>("robot_motion");
}


void RobotMotionTest::callback(const nav_msgs::Odometry& odom)
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

void RobotMotionTest::test()
{
    robotMotionServer(STOP);
    robotMotionServer(ROTATION);
    RandomDelay();
    robotMotionServer(STOP);
    robotMotionServer(VELOCITY);
    RandomDelay();
    RandomDelay();
}

bool RobotMotionTest::robotMotionServer(int modeID)
{
    ros::service::waitForService("robot_motion");
    req.request.motionModeID = 2;
    req.request.linear_x = 1.0;
    bool flag = client.call(req);
    if(flag){
        ROS_INFO("robot motion mode is suceess!!!");
        return flag;
    }
    else{
        ROS_ERROR("robot motion call is failed!!!");
        return flag;
    }
    
}

void RobotMotionTest::RandomDelay()
{
    ros::Rate r(generateRandomNumber(0.3,0.5));
    r.sleep();
    return;
}

double RobotMotionTest::generateRandomNumber(double minRange, double maxRange)
{
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
    ros::init(argc, argv, "robot_motion_test");
    RobotMotionTest test;
    while (ros::ok())
    {
        test.test();
    }
    
    
    ros::spin();

}
