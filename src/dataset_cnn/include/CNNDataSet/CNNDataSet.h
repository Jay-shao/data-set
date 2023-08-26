#ifndef _CNN_DataSet_H_
#define _CNN_DataSet_H_

/*ros include*/
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Bool.h>
#include <gazebo_msgs/SetModelState.h>
/*SQL*/
#include <vector>
#include <algorithm>
#include <string>
#include <sstream>
#include <fstream>
#include <time.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <iostream>
#include <random>
/*sensor_msgs*/
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
/*grid_map*/
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_core/TypeDefs.hpp"
#include "grid_map_ros/GridMapRosConverter.hpp"
#include "grid_map_ros/GridMapMsgHelpers.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_msgs/GridMap.h>
/*tf & nav_msgs*/
#include <nav_msgs/Odometry.h>
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
//image
#include <image_transport/image_transport.h>
#include <opencv2/imgcodecs.hpp>

using namespace std;
using namespace grid_map;
namespace CNNDataSet
{
    class DataSetForCNN
    {

    private:
        //ROS topic 相关变量
        ros::Publisher gridMapPublisher_;
        ros::Publisher robotStatePublisher_;
        ros::Publisher localMapPublisher_;
        ros::Publisher dataSetEndFlagPublisher_;
        ros::Publisher cmd_velPublisher_;
        ros::Subscriber imageSubscriber_;
        ros::Subscriber robotPoseSubscriber_;
        ros::Subscriber imuDataSubscriber_;
        //ros::Subscriber dataSetFlagSubscriber_;
        //ros::Subscriber robotVelocitySubscriber_;
        ros::Subscriber gridMapSubscriber_;

        ros::ServiceClient robotPositionClient_;
        

        //ROS句柄
        ros::NodeHandle& nh;

        //grid map
        //全局高度地图转换到GridMap类型的载体
        grid_map_msgs::GridMap mapMessage;

        //grid map地图数据对象
        grid_map::GridMap map_;

        //grid map初始化标志位
        bool mapInitialized_;

        //grid map地图参数
        double mapLengthX_;//地图长度
        double resolution_;//地图分辨率
        double minHeight_;//地图中最小高度值
        double maxHeight_;//地图中最小高度值
        std::string mapFrameId_;//地图的坐标系

        //! Name of the grid map topic.
        std::string gridMapTopic_;

        //! Path where to store the image.
        std::string filePath_;

        //图像发布话题名称
        std::string imageTopic_;

        //机器人位姿数据
        double robotAngleOfYaw;//机器人相对于世界坐标系的转角
        double robotPosition_x;//机器人在世界坐标系下的位置x分量
        double robotPosition_y;//机器人在世界坐标系下的位置y分量
        double robotPosition_z;//机器人在世界坐标系下的位置z分量

        //机器人运动变换向量
        Vector3 transformationVector;

        //IMU数据
        //机器人角速度
        double robotAngularVelocity_x;
        double robotAngularVelocity_y;
        double robotAngularVelocity_z;
        //机器人线速度
        double robotlinearVelocity_x;
        double robotlinearVelocity_y;
        double robotlinearVelocity_z;
        //IMU四元数
        double imu_x;
        double imu_y;
        double imu_z;
        double imu_w;
        //欧拉角
        double robotAngle_roll;
        double robotAngle_pitch;
        double robotAngle_yaw;

        //数据采集标志位
        bool dataSetFlag;

        //局部高度图采集窗口大小
        int localHeightMapWindow;

        //特征向量：机器人的振动矢量(需要记录到文档中的数据)
        double robotVibrationVector;

        //特征向量：到达目标点所需要的时间(需要记录到文档中的数据)
        double timeInterval;

        string filePathName;
        string fileName;

        gazebo_msgs::SetModelState objstate;

        geometry_msgs::Twist cmdVel;

        double robotLinearVelocity;

        double robotPositionBegin_x, robotPositionBegin_y;

        double robotPositionEnd_x, robotPositionEnd_y;

    public:

        //ROS句柄
        DataSetForCNN(ros::NodeHandle& nodehandle);
        //读取ROS参数服务器的参数
        bool readParameters();
        //读取高度图片话题回调函数
        void imageCallback(const sensor_msgs::Image &msg);
        //获取机器人位姿的回调函数
        void robotPoseCallback(const nav_msgs::Odometry& odom);
        //imu数据采集回调函数
        void imuDataSetCallback(const sensor_msgs::Imu& imu_data);
        //文件写入程序
        void txtWrite(string name_);
        //机器人状态采集程序
        bool robotStateDataSet();
        //局部高程地图采集程序
        bool localHeightMapSet();
        //机器人中心的局部高度地图发布函数
        void gridMapPub(grid_map::Index mapCenterIndex);
        //数据采集标志函数
        // bool dataSetFlag();
        void dataSetFlagCallback(const std_msgs::Bool& flag_msg);
        // 机器人速度话题回调函数
        void robotVelocityCallback(const geometry_msgs::Twist& msg);

        void gridMapCallback(const grid_map_msgs::GridMap& msg);

        void robotPositionUpdate();

        float randomNumber(int rangeMin, int rangeMax);

        void robotCmdVelPub();

        bool robotRotation(double targetAngle);

        void Mkdir(const char* pathName);

        void dataSetStart();

        bool IsRobotStateNormal();

        double generateRandomNumber(double minRange, double maxRange);

    };
} // namespace CNNDataSet




#endif