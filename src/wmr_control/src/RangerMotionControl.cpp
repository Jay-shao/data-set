#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <algorithm>
#include<nav_msgs/Odometry.h>
#include<geometry_msgs/Twist.h>
#include <iostream>

using namespace std;

class RangerMotioncontrol
{
private:
    ros::NodeHandle nh;

    // ros::Subscriber leftBackWheelVelocityController;
    // ros::Subscriber rightBackWheelVelocityController;
    // ros::Subscriber leftFrontWheelVelocityController;
    // ros::Subscriber rightFrontWheelVelocityController;

    // ros::Subscriber leftBackWheelPositionController;
    // ros::Subscriber rightBackWheelPositionController;
    // ros::Subscriber leftFrontWheelPositionController;
    // ros::Subscriber rightFrontWheelPositionController;

    ros::Subscriber cmd_Mobile_base;

    ros::Publisher leftBackWheelVelocityController;
    ros::Publisher rightBackWheelVelocityController;
    ros::Publisher leftFrontWheelVelocityController;
    ros::Publisher rightFrontWheelVelocityController;

    ros::Publisher leftBackWheelPositionController;
    ros::Publisher rightBackWheelPositionController;
    ros::Publisher leftFrontWheelPositionController;
    ros::Publisher rightFrontWheelPositionController;

    std_msgs::Float64 leftBackVelocityValue;
    std_msgs::Float64 rightBackVelocityValue;
    std_msgs::Float64 leftFrontVelocityValue;
    std_msgs::Float64 rightFrontVelocityValue;

    std_msgs::Float64 leftBackPositonValue;
    std_msgs::Float64 rightBackPositonValue;
    std_msgs::Float64 leftFrontPositonValue;
    std_msgs::Float64 rightFrontPositonValue;

    double r_x = 0.2455;//m  490.11
    double r_y = 0.186;//m  372.07
    double r = 0.308;


public:

    RangerMotioncontrol();
    ~RangerMotioncontrol();

    // // wheel velocity massage call back
    // void GetWheelLBVelocityCallback(std_msgs::Float64 &LBVelocity);
    // void GetWheelRBVelocityCallback(std_msgs::Float64 &RBVelocity);
    // void GetWheelLFVelocityCallback(std_msgs::Float64 &LFVelocity);
    // void GetWheelRFVelocityCallback(std_msgs::Float64 &RFVelocity);

    // ///wheel position massage call back
    // void GetWheelLBPositionCallback(std_msgs::Float64 &LBPosition);
    // void GetWheelRBPositionCallback(std_msgs::Float64 &RBPosition);
    // void GetWheelLFPositionCallback(std_msgs::Float64 &LFPosition);
    // void GetWheelRFPositionCallback(std_msgs::Float64 &RFPosition);

    void cmd_velCallback(const geometry_msgs::Twist& cmd_vel);

    int wheelPositionSingle(double v_x, double v_y);

};

RangerMotioncontrol::RangerMotioncontrol()
{

    cmd_Mobile_base = nh.subscribe("/ranger/cmd_vel", 20, &RangerMotioncontrol::cmd_velCallback, this);

    leftBackWheelVelocityController = nh.advertise<std_msgs::Float64>("/ranger/wheel_lb_velocity_controller/command", 20, true);
    rightBackWheelVelocityController = nh.advertise<std_msgs::Float64>("/ranger/wheel_rb_velocity_controller/command", 20, true);
    leftFrontWheelVelocityController = nh.advertise<std_msgs::Float64>("/ranger/wheel_lf_velocity_controller/command", 20, true);
    rightFrontWheelVelocityController = nh.advertise<std_msgs::Float64>("/ranger/wheel_rf_velocity_controller/command", 20, true);

    leftBackWheelPositionController = nh.advertise<std_msgs::Float64>("/ranger/lb_position_controller/command", 20, true);
    rightBackWheelPositionController = nh.advertise<std_msgs::Float64>("/ranger/rb_position_controller/command", 20, true);
    leftFrontWheelPositionController = nh.advertise<std_msgs::Float64>("/ranger/lf_position_controller/command", 20, true);
    rightFrontWheelPositionController = nh.advertise<std_msgs::Float64>("/ranger/rf_position_controller/command", 20, true);

    /// wheel velocity massage subscriber
    // leftBackWheelVelocityController = nh.subscribe("/ranger/wheel_lb_velocity_controller/command", 20, &RangerMotioncontrol::GetWheelLBVelocityCallback, this);
    // rightBackWheelVelocityController = nh.subscribe("/ranger/wheel_rb_velocity_controller/command", 20, &RangerMotioncontrol::GetWheelRBVelocityCallback, this);
    // leftFrontWheelVelocityController = nh.subscribe("/ranger/wheel_lf_velocity_controller/command", 20, &RangerMotioncontrol::GetWheelLFVelocityCallback, this);
    // rightFrontWheelVelocityController = nh.subscribe("/ranger/wheel_rf_velocity_controller/command", 20, &RangerMotioncontrol::GetWheelRFVelocityCallback, this);

    /// wheel position massage subscriber
    // leftBackWheelPositionController = nh.subscribe("/ranger/lb_position_controller/command", 20, &RangerMotioncontrol::GetWheelLBPositionCallback, this);
    // rightBackWheelPositionController = nh.subscribe("/ranger/rb_position_controller/command", 20, &RangerMotioncontrol::GetWheelRBPositionCallback, this);
    // leftFrontWheelPositionController = nh.subscribe("/ranger/lf_position_controller/command", 20, &RangerMotioncontrol::GetWheelLFPositionCallback, this);
    // rightFrontWheelPositionController = nh.subscribe("/ranger/rf_position_controller/command", 20, &RangerMotioncontrol::GetWheelRFPositionCallback, this);
}

// int RangerMotioncontrol::wheelPositionSingle(double v_x, double v_y)
// {
//     if(v_x > 0 && v_y > 0)
//     {
//         return 
//     }
// }

/// @brief left back wheel velocity call back function
/// @param LBVelocity
void RangerMotioncontrol::cmd_velCallback(const geometry_msgs::Twist& cmd_vel){

    if(cmd_vel.linear.x == 0.0 && cmd_vel.linear.y == 0.0 && cmd_vel.linear.z == 0.0 && cmd_vel.angular.z != 0.0)
    {
        if (cmd_vel.angular.z > 0)
        {
            leftFrontVelocityValue.data = -0.5;
            leftBackVelocityValue.data = -0.5;
            rightBackVelocityValue.data = 0.5;
            rightFrontVelocityValue.data = 0.5;

            leftFrontPositonValue.data = 0.7853981634;
            leftBackPositonValue.data = -0.7853981634;
            rightBackPositonValue.data = 0.7853981634;
            rightFrontPositonValue.data = -0.7853981634;
        }
        else{
            leftFrontVelocityValue.data = 0.5;
            leftBackVelocityValue.data = 0.5;
            rightBackVelocityValue.data = -0.5;
            rightFrontVelocityValue.data = -0.5;

            leftFrontPositonValue.data = 0.7853981634;
            leftBackPositonValue.data = -0.7853981634;
            rightBackPositonValue.data = 0.7853981634;
            rightFrontPositonValue.data = -0.7853981634;
        }
        
    }
    else if (cmd_vel.linear.x == 0 && cmd_vel.linear.y == 0 && cmd_vel.linear.z == 0 && cmd_vel.angular.x == 0 && cmd_vel.angular.y == 0 && cmd_vel.angular.z == 0)
    {
            leftFrontVelocityValue.data = 0.0;
            leftBackVelocityValue.data = 0.0;
            rightBackVelocityValue.data = 0.0;
            rightFrontVelocityValue.data = 0.0;

            leftFrontPositonValue.data = 0.0;
            leftBackPositonValue.data = 0.0;
            rightBackPositonValue.data = 0.0;
            rightFrontPositonValue.data = 0.0;
    }
    else
    {
        double v1_x = cmd_vel.linear.x - cmd_vel.angular.z * r_y, v1_y = cmd_vel.linear.y + cmd_vel.angular.z * r_x;
        double v2_x = cmd_vel.linear.x - cmd_vel.angular.z * r_y, v2_y = cmd_vel.linear.y - cmd_vel.angular.z * r_x;
        double v3_x = cmd_vel.linear.x + cmd_vel.angular.z * r_y, v3_y = cmd_vel.linear.y - cmd_vel.angular.z * r_x;
        double v4_x = cmd_vel.linear.x + cmd_vel.angular.z * r_y, v4_y = cmd_vel.linear.y + cmd_vel.angular.z * r_x;

        leftFrontVelocityValue.data = sqrt(pow(v1_x, 2)+pow((v1_y),2));
        leftBackVelocityValue.data = sqrt(pow((v2_x), 2)+pow((v2_y),2));
        rightBackVelocityValue.data = sqrt(pow((v3_x), 2)+pow((v3_y),2));
        rightFrontVelocityValue.data = sqrt(pow((v4_x), 2)+pow((v4_y),2));
        
        // leftFrontVelocityValue.data = sqrt(pow((cmd_vel.linear.x - cmd_vel.angular.z * r_y), 2)+pow((cmd_vel.linear.y + cmd_vel.angular.z * r_x),2));
        // leftBackVelocityValue.data = sqrt(pow((cmd_vel.linear.x - cmd_vel.angular.z * r_y), 2)+pow((cmd_vel.linear.y - cmd_vel.angular.z * r_x),2));
        // rightBackVelocityValue.data = sqrt(pow((cmd_vel.linear.x + cmd_vel.angular.z * r_y), 2)+pow((cmd_vel.linear.y - cmd_vel.angular.z * r_x),2));
        // rightFrontVelocityValue.data = sqrt(pow((cmd_vel.linear.x + cmd_vel.angular.z * r_y), 2)+pow((cmd_vel.linear.y + cmd_vel.angular.z * r_x),2));

        leftFrontPositonValue.data = -atan(v1_y / v1_x);
        leftBackPositonValue.data = -atan(v2_y / v2_x);
        rightBackPositonValue.data = -atan(v3_y / v3_x);
        rightFrontPositonValue.data = -atan(v4_y / v4_x);

        // leftFrontPositonValue.data = acos((cmd_vel.linear.x - cmd_vel.angular.z * r_y) / leftFrontVelocityValue.data);
        // leftBackPositonValue.data = acos((cmd_vel.linear.x - cmd_vel.angular.z * r_y) / leftBackVelocityValue.data);
        // rightBackPositonValue.data = acos((cmd_vel.linear.x + cmd_vel.angular.z * r_y) / rightBackVelocityValue.data);
        // rightFrontPositonValue.data = acos((cmd_vel.linear.x + c

    }
    leftFrontWheelVelocityController.publish(leftFrontVelocityValue);
    leftBackWheelVelocityController.publish(leftBackVelocityValue);
    rightBackWheelVelocityController.publish(rightBackVelocityValue);
    rightFrontWheelVelocityController.publish(rightFrontVelocityValue);

    leftFrontWheelPositionController.publish(leftFrontPositonValue);
    leftBackWheelPositionController.publish(leftBackPositonValue);
    rightBackWheelPositionController.publish(rightBackPositonValue);
    rightFrontWheelPositionController.publish(rightFrontPositonValue);

}




RangerMotioncontrol::~RangerMotioncontrol()
{
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "RangerMotionControl");
    RangerMotioncontrol ranger;

    ros::spin();
    return 0;
}