#include <ros/ros.h>
#include <sensor_msgs/Imu.h>


void IMUDataCallback(const sensor_msgs::Imu& imu_data)
{
    double robotlinearVelocity_x = imu_data.linear_acceleration.x;
    double robotlinearVelocity_y = imu_data.linear_acceleration.y;
    double robotlinearVelocity_z = imu_data.linear_acceleration.z;

    ROS_INFO("x is:%.2f, y is:%.2f, z is:%.2f", robotlinearVelocity_x,robotlinearVelocity_y,robotlinearVelocity_z);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "IMU_test");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/imu", 1, &IMUDataCallback);
    ros::spin();
    return 0;
}
