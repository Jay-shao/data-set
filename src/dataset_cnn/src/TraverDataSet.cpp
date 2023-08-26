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

//srv
#include "wmr_control/robot_move.h"
#include "grid_map_msgs/MyGridMap.h"

#define STOP 1
#define VELOCITY 2
#define ROTATION 3

using namespace std;
using namespace grid_map;
using namespace grid_map_msgs;

class TraverDataSet
{
private:
    ros::NodeHandle nh;
    //ros::Subscriber imageSubscriber_;
    ros::Subscriber robotPoseSubscriber_;
    ros::Subscriber imuDataSubscriber_;
    //ros::Subscriber gridMapSubscriber_;
    ros::Subscriber cmdVelSubscriber_;
    ros::ServiceClient robotMotionClient_;
    ros::ServiceClient robotPositionClient_;
    ros::ServiceClient gridMapPubClient_;

    ros::Timer IsNormalTimer_;

    string filePathName;
    string fileName;

    //局部高度图采集窗口大小
    int localHeightMapWindow;

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

    //特征向量：机器人的振动矢量(需要记录到文档中的数据)
    double robotVibrationVector;

    //特征向量：到达目标点所需要的时间(需要记录到文档中的数据)
    double timeInterval;

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

    gazebo_msgs::SetModelState objstate;

    wmr_control::robot_move robotMoveReq;

    grid_map_msgs::MyGridMap gridMapReq;

    double robotLinearVelocity;

    double robotPositionBegin_x, robotPositionBegin_y;

    double robotPositionEnd_x, robotPositionEnd_y;

    bool normalState;

    

public:
    TraverDataSet();

    //读取高度图片话题回调函数
    void imageCallback(const sensor_msgs::Image &msg);
    //获取机器人位姿的回调函数
    void robotPoseCallback(const nav_msgs::Odometry& odom);
    //imu数据采集回调函数
    void imuDataSetCallback(const sensor_msgs::Imu& imu_data);
    // 机器人速度话题回调函数
    void robotVelocityCallback(const geometry_msgs::Twist& msg);
    // grid map地图订阅回调函数
    void gridMapCallback(const grid_map_msgs::GridMap& msg);
    // 机器人位置生成函数
    void robotPositionUpdate();
    //文件写入程序
    void txtWrite(string name_);
    //新建文件夹程序
    void Mkdir(const char* pathName);
    //机器人状态采集程序
    bool robotStateDataSet();
    //局部高程地图采集程序
    bool localHeightMapSet();
    //参数服务器初始化函数
    bool readParameters();
    //随机数生成函数
    double generateRandomNumber(double minRange, double maxRange);
    //机器人运动模式客户端函数
    bool robotMotionServer(int modeID, double linear);
    //grid map 发布客户端函数
    void gridMapPubServer(grid_map::Index mapCenterIndex);
    //判断机器人运动是否正常的函数
    bool IsRobotStateNormal();

    void NormalStateTimer();

    void TraverDataSetMain();

    void doStateCheck(const ros::TimerEvent &event);

    void RandomDelay();

    void robotStateDataSet(double positionBegin_x, double positionBegin_y, double timeBegin, double angleBegin, double velocity, vector<double> rollOrin, vector<double> pitchOrin, double rollSum, double pitchSum, double N);


};

// : map_(grid_map::GridMap({"elevation"})),
//     mapInitialized_(false)
TraverDataSet::TraverDataSet()

{
    filePathName = "/home/jay/jay/dataSet_20230823/Jay_ws/src/Data/";
    fileName = "CNNDataSet";
    localHeightMapWindow = 40;
    normalState = true;
    //dataSetFlag = false;
    readParameters();
    map_.setBasicLayers({"elevation"});
    //imageSubscriber_ = nh.subscribe("/image_publisher/image", 10, &TraverDataSet::imageCallback, this);
    robotPoseSubscriber_ = nh.subscribe("/odom", 10, &TraverDataSet::robotPoseCallback, this);
    imuDataSubscriber_ = nh.subscribe("/imu", 10, &TraverDataSet::imuDataSetCallback, this);
    //gridMapSubscriber_ = nh.subscribe(gridMapTopic_, 10, &TraverDataSet::gridMapCallback, this);
    cmdVelSubscriber_ = nh.subscribe("/ranger/cmd_vel", 10, &TraverDataSet::robotVelocityCallback, this);
    robotMotionClient_ = nh.serviceClient<wmr_control::robot_move>("/robot_motion");
    robotPositionClient_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    gridMapPubClient_ = nh.serviceClient<grid_map_msgs::MyGridMap>("/gridmap_to_image");
    ROS_INFO("Subscribed to %s", nh.resolveName(gridMapTopic_).c_str());
}

//参数服务器设置
bool TraverDataSet::readParameters()
{
    //在这个部分添加判断，Deug
    nh.param("image_topic", imageTopic_, std::string("/image"));
    nh.param("resolution", resolution_, 0.05);
    nh.param("min_height", minHeight_, 0.0);
    nh.param("max_height", maxHeight_, 1.0);
    nh.param("grid_map_topic", gridMapTopic_, std::string("/cnn_data_set/local_height_map"));
    nh.param("image_file", filePath_, std::string("/home/jay/jay/dataSet_20230823/Jay_ws/src/image/heightmap.png"));
    return true;
}

void TraverDataSet::RandomDelay()
{
    ros::Rate r(generateRandomNumber(0.2,0.5));
    r.sleep();
    return;
}

void TraverDataSet::robotVelocityCallback(const geometry_msgs::Twist& msg)
{
    ROS_ERROR("ni ta ma neng buneng jinlai le ?2");
    robotLinearVelocity = msg.linear.x;
}

//高度地图转换为grid map地图
// void TraverDataSet::imageCallback(const sensor_msgs::Image& msg)
// {
//     ROS_ERROR("ni ta ma neng buneng jinlai le ?1");
//     if (!mapInitialized_) {
//         grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
//         ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
//                 map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
//         mapInitialized_ = true;
//     }
//     //设置grid map基础层的名称以及地图数据
//     grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);

//     //添加颜色层(应该只是用于显示，没有具体的作用，不影响数据采集)
//     grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);

//     // 发布转换完成的grid map地图
//     grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
//     //gridMapPublisher_.publish(mapMessage);
// }

void TraverDataSet::robotPoseCallback(const nav_msgs::Odometry& odom)
{
    ROS_ERROR("ni ta ma neng buneng jinlai le ?");
    tf::Quaternion quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    robotAngleOfYaw = yaw;

    robotPosition_x = odom.pose.pose.position.x;
    cout<<"robotPosition_x is :"<<robotPosition_x<<endl;
    robotPosition_y = odom.pose.pose.position.y;
    robotPosition_z = odom.pose.pose.position.z;
}

void TraverDataSet::imuDataSetCallback(const sensor_msgs::Imu& imu_data)
{
    ROS_ERROR("ni ta ma neng buneng jinlai le ?3");
    robotAngularVelocity_x = imu_data.angular_velocity.x;
    robotAngularVelocity_y = imu_data.angular_velocity.y;
    robotAngularVelocity_z = imu_data.angular_velocity.z;

    robotlinearVelocity_x = imu_data.linear_acceleration.x;
    robotlinearVelocity_y = imu_data.linear_acceleration.y;
    robotlinearVelocity_z = imu_data.linear_acceleration.z;

    imu_x = imu_data.orientation.x;
    imu_y = imu_data.orientation.y;
    imu_z = imu_data.orientation.z;
    imu_w = imu_data.orientation.w;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(imu_data.orientation, quat);
    tf::Matrix3x3(quat).getRPY(robotAngle_roll, robotAngle_pitch, robotAngle_yaw);
}

// 将局部高度grid map地图转换成image图像
// void TraverDataSet::gridMapCallback(const grid_map_msgs::GridMap& msg)
// {
//     ROS_ERROR("ni ta ma neng buneng jinlai le ?4");
//     ROS_INFO("Saving map received from: %s to file %s.", nh.resolveName(gridMapTopic_).c_str(), fileName.c_str());
//     grid_map::GridMap map;
//     cv_bridge::CvImage image;
//     grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
//     grid_map::GridMapRosConverter::toCvImage(map,"elevation", sensor_msgs::image_encodings::MONO8, image);
//     bool success = cv::imwrite(fileName.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
//     ROS_INFO("Success writing image: %s", success?"true":"false");
// }

void TraverDataSet::robotPositionUpdate()
{
    ROS_INFO("Reset robot position..........");
    objstate.request.model_state.model_name = "mobile_base";
    objstate.request.model_state.pose.position.x = generateRandomNumber(0,20);
    objstate.request.model_state.pose.position.y = generateRandomNumber(0,20);
    objstate.request.model_state.pose.position.z = generateRandomNumber(0,5);
    objstate.request.model_state.pose.orientation.w = 1.0;
    objstate.request.model_state.pose.orientation.x = 0.0;
    objstate.request.model_state.pose.orientation.y = 0.0;
    objstate.request.model_state.pose.orientation.z = 0.0;
    objstate.request.model_state.twist.linear.x = 0.0;
    objstate.request.model_state.twist.linear.y = 0.0;
    objstate.request.model_state.twist.linear.z = 0.0;
    objstate.request.model_state.twist.angular.x = 0.0;
    objstate.request.model_state.twist.angular.y = 0.0;
    objstate.request.model_state.twist.angular.z = 0.0;
    objstate.request.model_state.reference_frame = "world";

    robotPositionClient_.call(objstate);
    ROS_INFO("Robot position is updated!");

    //return true;
}

double TraverDataSet::generateRandomNumber(double minRange, double maxRange)
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

void TraverDataSet::Mkdir(const char* pathName)
{
    
    int isCreate = mkdir(pathName,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    if( !isCreate )
    ROS_WARN("create path");
    else
    ROS_ERROR("create path failed! error code : %s",isCreate,pathName);
}

bool TraverDataSet::robotMotionServer(int modeID, double linear)
{
    ros::service::waitForService("robot_motion");
    robotMoveReq.request.motionModeID = modeID;
    robotMoveReq.request.linear_x = linear;
    bool flag = robotMotionClient_.call(robotMoveReq);
    if(flag){
        
        return flag;
    }
    else{
        ROS_ERROR("robot motion call is failed!!!");
        return flag;
    }
    
}

void TraverDataSet::gridMapPubServer(grid_map::Index mapCenterIndex)
{
    ros::service::waitForService("gridmap_to_image");
    gridMapReq.request.index_1 = mapCenterIndex(0);
    gridMapReq.request.index_2 = mapCenterIndex(1);
    grid_map_msgs::GridMap theMap;
    grid_map::GridMapRosConverter::toMessage(map_, theMap);
    gridMapReq.request.map = theMap;
    gridMapReq.request.resolution = resolution_;
    gridMapReq.request.window = localHeightMapWindow;
    bool flag = gridMapPubClient_.call(gridMapReq);

}

//该函数有一些问题，不够完善。
// 添加IMU位姿的判断
bool TraverDataSet::IsRobotStateNormal()
{
    if( (abs(robotAngle_roll)>=1.57) || (abs(robotAngle_pitch)>=1.57) )
    {
        return false;
    }else
    {
        return true;
    }
    
}

void TraverDataSet::txtWrite(string name_)
{
    ofstream file;
    string txtName = "data.txt";
    txtName = name_ + txtName;//txtName is txt path
    file.open(txtName, std::ios::out | std::ios::app);
    file << "robotTransformPostionX:"
            << "\t"
            << "robotTransformPostionY:"
            << "\t"
            << "robotTransformAngle:"
            << "\t"
            << "robotAngle:"
            << "\t"
            << "robotVelocity:" 
            << "\t"
            << "robotVibrationVector"
            << "\t"
            << "timeInterval"
            << "\t" << endl;
    file << setprecision(16)
            << transformationVector(0)
            << "\t"
            << transformationVector(1)
            << "\t"
            << transformationVector(2)
            << "\t"
            << robotAngleOfYaw
            << "\t"
            << robotLinearVelocity
            << "\t"
            << robotVibrationVector
            << "\t"
            << timeInterval
            << "\t" << endl;
    file.close(); 
    ROS_INFO("Data is writed to txt file!");
}

void TraverDataSet::NormalStateTimer()
{
    IsNormalTimer_ = nh.createTimer(ros::Duration(0.5), &TraverDataSet::doStateCheck, this);
}

void TraverDataSet::doStateCheck(const ros::TimerEvent &event)
{
    double positionTemp_x = robotPosition_x, positionTemp_y = robotPosition_y;
    if(((abs(robotAngle_roll)>=1.57) || (abs(robotAngle_pitch)>=1.57)) || ( ((robotPosition_x >= (positionTemp_x - 0.01)) || (robotPosition_x <= (positionTemp_x + 0.01))) && ((robotPosition_y >= (positionTemp_y - 0.01)) || (robotPosition_y <= (positionTemp_y + 0.01)))))
    {
        normalState = false;
        robotPositionUpdate();
    }else{
        normalState = true;
    }
}

void TraverDataSet::TraverDataSetMain()
{
    
    while (!IsRobotStateNormal())
    {
        robotPositionUpdate();
    }

    //NormalStateTimer();


    //重置机器人运动状态
    robotMotionServer(STOP,0);
    robotMotionServer(ROTATION,0);
    RandomDelay();


    double beginPosition_x = robotPosition_x, beginPosition_y = robotPosition_y, angleBegin = robotAngleOfYaw, N = 0.0;
    double moveDistance = 0.0;
    double linear_x = generateRandomNumber(0,5);
    vector<double> rollOrin, pitchOrin;
    double rollSum = 0.0, pitchSum = 0.0;
    ros::Time timeBegin = ros::Time::now();
    double Time_begin = timeBegin.toSec();


    //建立文件夹
    time_t timep;

    static char name[256] = {0};

    time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep

    strftime( name, sizeof(name), "%Y.%m.%d %H-%M-%S",localtime(&timep) );

    string timeName = name;

    fileName = filePathName + fileName + timeName;//fileName is dir path

    const char* pathName = fileName.data();

    Mkdir(pathName);
    
    bool FLAG = true;
    // if(localHeightMapSet())
    if(FLAG)
    {
        ROS_ERROR("is in?");
        while((moveDistance < generateRandomNumber(0.5, 0.8) )&& normalState)
        {
            //ROS_ERROR("robotPosition_x is %.4f, robotPosition_y is %.4f, beginPosition_x is %.4f, beginPosition_y is %.4f",robotPosition_x,robotPosition_y,beginPosition_x,beginPosition_y);
            
            robotMotionServer(VELOCITY,linear_x);
            moveDistance = sqrt((pow ((robotPosition_x - beginPosition_x), 2 )) + (pow( (robotPosition_y - beginPosition_y), 2 )) );
            //cout<<"moveDistance is :"<<moveDistance<<", robotPosition_x is :"<<robotPosition_x<<", beginPosition_x is :"<<beginPosition_x<<endl;
            rollOrin.push_back(robotlinearVelocity_x);
            pitchOrin.push_back(robotlinearVelocity_y);
            rollSum += robotlinearVelocity_x;
            pitchSum += robotlinearVelocity_y;
            N +=  1.0;
        }
        //停车
        robotMotionServer(STOP,0);

        if(!normalState)
        {
        
            //数据计算
            robotStateDataSet(beginPosition_x, beginPosition_y, Time_begin, angleBegin, linear_x, rollOrin, pitchOrin, rollSum, pitchSum, N);
            //文档写入
            txtWrite(fileName);

        }
        else{
            //机器人状态异常，可通过性代价为0
            robotStateDataSet(beginPosition_x, beginPosition_y, Time_begin, angleBegin, linear_x, rollOrin, pitchOrin, rollSum, pitchSum, N);
            robotVibrationVector = 0.0;
            timeInterval = 0.0;
            return;
            ROS_ERROR("robot is'nt normal state!!! spin agin!!!!!!");
        }

    }
    
    //计算数据，数据写入文档   

}

void TraverDataSet::robotStateDataSet(double positionBegin_x, double positionBegin_y, double timeBegin, double angleBegin, double velocity, vector<double> rollOrin, vector<double> pitchOrin, double rollSum, double pitchSum, double N)
{
    ros::Time timeEnd = ros::Time::now();
    double Time_end = timeEnd.toSec();
    double rollAverage = 0.0, pitchAverage = 0.0;
    // double positionEnd_x = robotPosition_x, positionEnd_y = robotPosition_y, endAngle = robotAngleOfYaw;
    timeInterval = Time_end - timeBegin;
    rollAverage = rollSum / N;
    pitchAverage = pitchSum / N;
    transformationVector(0) = robotPosition_x - positionBegin_x;
    transformationVector(1) = robotPosition_y - positionBegin_y;
    transformationVector(2) = robotAngleOfYaw - angleBegin;
    if(!rollOrin.empty())
    {
        rollSum = 0.0;
        pitchSum = 0.0;
        for (int i = 0; i < rollOrin.size(); i++)
        {
            rollSum += pow((rollOrin[i] - rollAverage), 2);
            pitchSum += pow((pitchOrin[i] - pitchAverage), 2);
        }
        robotVibrationVector = (sqrt(rollSum * pitchSum)) / N;
        
    }
    else{
        ROS_ERROR("rollOrin is empty");
    }


}

bool TraverDataSet::localHeightMapSet()
{
    //定义一个position类的对象存储机器人中心所在位置
    grid_map::Position robotPosition2d(robotPosition_x, robotPosition_y);//确定一下：odom得到的robotPosition_x和y与grid_map中的位置能不能对应的上
    //运动grid_map中的getIndex()函数，获取局部地图中心所在位置的网格索引
    grid_map::Index localHeightMapIndex;
    ROS_ERROR("is here?");
    map_.getIndex(robotPosition2d,localHeightMapIndex);
    ROS_ERROR("REALLY?");
    //以网格索引向周围扩散loaclHeightMapWindow范围
    if ((localHeightMapIndex(0) < (localHeightMapWindow/2) || localHeightMapIndex(1) < (localHeightMapWindow/2))||(localHeightMapIndex(0) > (map_.getSize()(0)-localHeightMapWindow/2) || localHeightMapIndex(0) > (map_.getSize()(1)-localHeightMapWindow/2)))
    {
        return false;
    }
    else{
        gridMapPubServer(localHeightMapIndex);
        return true;
    }    
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "traver_dataset");
    // ros::MultiThreadedSpinner spinner(4); 
    TraverDataSet Traver;
    Traver.TraverDataSetMain();
    // spinner.spin();
    ros::spin();
    return 0;
}
