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
class CNNDataSet
{

private:

    ros::NodeHandle nh;
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
    // ros::NodeHandle& nh;

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
    CNNDataSet();
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

CNNDataSet::CNNDataSet()
    : map_(grid_map::GridMap({"elevation"})),
      mapInitialized_(false)
      {
        filePathName = "/home/jay/jay/Jay_ws/src/Data";
        fileName = "CNNDataSet";
        localHeightMapWindow = 40;
        dataSetFlag = false;
        readParameters();
        map_.setBasicLayers({"elevation"});
        imageSubscriber_ = nh.subscribe(imageTopic_, 1, &CNNDataSet::imageCallback, this);
        gridMapPublisher_ = nh.advertise<grid_map_msgs::GridMap>("/ranger/global_height_map", 1, true);
        localMapPublisher_ = nh.advertise<grid_map_msgs::GridMap>("/ranger/local_height_map", 1, true);
        dataSetEndFlagPublisher_ = nh.advertise<std_msgs::Bool>("/data_set_end_flag", 1, true);
        robotPoseSubscriber_ = nh.subscribe("/odom", 1, &CNNDataSet::robotPoseCallback, this);
        robotStatePublisher_ = nh.advertise<geometry_msgs::Vector3>("/ranger/robot_state", 1, true);
        imuDataSubscriber_ = nh.subscribe("/imu", 1, &CNNDataSet::imuDataSetCallback, this);
        //dataSetFlagSubscriber_ = nh.subscribe("/data_set_flag", 1, &DataSetForCNN::dataSetFlagCallback, this);
        //robotVelocitySubscriber_ = nh.subscribe("/cmd_vel", 1, &DataSetForCNN::robotVelocityCallback, this);
        gridMapSubscriber_ = nh.subscribe(gridMapTopic_, 1, &CNNDataSet::gridMapCallback, this);
        robotPositionClient_ = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
        cmd_velPublisher_ = nh.advertise<geometry_msgs::Twist>("/ranger/cmd_vel", 1, true);
        ROS_INFO("Subscribed to %s", nh.resolveName(gridMapTopic_).c_str());
      }

    //参数服务器设置
    bool CNNDataSet::readParameters()
    {
        //在这个部分添加判断，Deug
        nh.param("image_topic", imageTopic_, std::string("/image"));
        nh.param("resolution", resolution_, 0.05);
        nh.param("min_height", minHeight_, 0.0);
        nh.param("max_height", maxHeight_, 1.0);
        nh.param("grid_map_topic", gridMapTopic_, std::string("/ranger/local_height_map"));
        nh.param("image_file", filePath_, std::string("/home/jay/jay/Jay_ws/src/Data"));
        return true;
    }

    //高度地图转换为grid map地图
    void CNNDataSet::imageCallback(const sensor_msgs::Image& msg)
    {
        if (!mapInitialized_) {
            grid_map::GridMapRosConverter::initializeFromImage(msg, resolution_, map_);
            ROS_INFO("Initialized map with size %f x %f m (%i x %i cells).", map_.getLength().x(),
                    map_.getLength().y(), map_.getSize()(0), map_.getSize()(1));
            mapInitialized_ = true;
        }
        //设置grid map基础层的名称以及地图数据
        grid_map::GridMapRosConverter::addLayerFromImage(msg, "elevation", map_, minHeight_, maxHeight_);

        //添加颜色层(应该只是用于显示，没有具体的作用，不影响数据采集)
        grid_map::GridMapRosConverter::addColorLayerFromImage(msg, "color", map_);

        // 发布转换完成的grid map地图
        grid_map::GridMapRosConverter::toMessage(map_, mapMessage);
        gridMapPublisher_.publish(mapMessage);
    }

    void CNNDataSet::robotPoseCallback(const nav_msgs::Odometry& odom)
    {
        tf::Quaternion quat;
        double roll, pitch, yaw;
		tf::quaternionMsgToTF(odom.pose.pose.orientation, quat);
		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        robotAngleOfYaw = yaw;
        robotPosition_x = odom.pose.pose.position.x;
        ROS_ERROR("robotPosition_x is :%.4f |||||| %.4f",robotPosition_x,odom.pose.pose.position.x);
        robotPosition_y = odom.pose.pose.position.y;
        robotPosition_z = odom.pose.pose.position.z;
    }

    bool CNNDataSet::robotStateDataSet()
    {
        //计算机器人的振动矢量（到达目标点的时间-想一想要不要写到这个函数中）
        //这里得写一个类似于延时的程序，因为局部高度地图只需要发布一次，但是机器人的状态需要采集整个过程的数据
        //因此到达目标点的时间可以写在这个函数中
        //需要一个机器人到达目标点的起始和标志位
        if (localHeightMapSet())
        {
            //这部分写一个while循环，判断条件就是数据采集开始标志位，循环采集角加速度
            //出循环之后统一计算
            //while上面写一个获取当前时间的变量，然后出循环之后再获取一下当前时间，然后两者取差值
            ros::Time Time_t = ros::Time::now();
            double currentTime = Time_t.toSec();
            time_t timep;
            static char name[256] = {0};
            time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep
            strftime( name, sizeof(name), "%Y.%m.%d %H-%M-%S.txt",localtime(&timep) );
            string timeName = name;
            fileName = filePathName + fileName + timeName;//fileName is dir path
            const char* pathName = fileName.data();
            Mkdir(pathName);

            std_msgs::Bool endFlagMsg;
            double sumOfroll = 0.0, sumOfpitch = 0.0, averageOfroll = 0.0, averageOfpitch = 0.0, N=0.0;
            vector<double> rollOrin, pitchOrin;
            // double beginPosition_x = robotPosition_x, beginPosition_y = robotPosition_y, beginAngle = robotAngleOfYaw;
            double beginAngle = robotAngleOfYaw;
            robotPositionEnd_x = robotPosition_x;
            robotPositionEnd_y = robotPosition_y;
            double distance = sqrt((pow(robotPositionBegin_x, 2)+pow(robotPositionBegin_y, 2)));
            ros::Time begin = ros::Time::now();
            double Time_begin = begin.toSec();
            while(distance >= 0.3 && distance >= 0.6)
            {
                rollOrin.push_back(robotlinearVelocity_x);
                pitchOrin.push_back(robotlinearVelocity_y);
                sumOfroll +=robotlinearVelocity_x;
                sumOfpitch += robotlinearVelocity_y;
                N += 1.0;
            }
            ros::Time end = ros::Time::now();
            double Time_end = end.toSec();
            //计算时间间隔
            timeInterval = Time_end - Time_begin;  
            averageOfroll = sumOfroll / N;
            averageOfpitch = sumOfpitch / N; 
            // double endPosition_x = robotPosition_x, endPosition_y = robotPosition_y, endAngle = robotAngleOfYaw;
            double endAngle = robotAngleOfYaw;
            transformationVector(0) = robotPositionEnd_x - robotPositionBegin_x;
            transformationVector(1) = robotPositionEnd_y - robotPositionBegin_y;
            transformationVector(2) = endAngle - beginAngle;
            if (!rollOrin.empty())
            {
                sumOfpitch = 0.0;
                sumOfroll = 0.0;
                for (int i = 0; i < rollOrin.size(); i++)
                {
                    sumOfroll += pow((rollOrin[i] - averageOfroll), 2);
                    sumOfpitch += pow((pitchOrin[i] - averageOfpitch), 2);
                }
                robotVibrationVector = (sqrt(sumOfroll * sumOfpitch)) / N;
                endFlagMsg.data = true;
                dataSetFlag = false;
                dataSetEndFlagPublisher_.publish(endFlagMsg);
                return true;
            }
            else{
                ROS_ERROR("rollOrin is empty");
                return false;
            }

        }
        else{
            return false;
        }
        
    }

    bool CNNDataSet::localHeightMapSet()
    {
        //定义一个position类的对象存储机器人中心所在位置
        grid_map::Position robotPosition2d(robotPosition_x, robotPosition_y);//确定一下：odom得到的robotPosition_x和y与grid_map中的位置能不能对应的上
        //运动grid_map中的getIndex()函数，获取局部地图中心所在位置的网格索引
        grid_map::Index localHeightMapIndex;
        map_.getIndex(robotPosition2d,localHeightMapIndex);
        //以网格索引向周围扩散loaclHeightMapWindow范围
        if ((localHeightMapIndex(0) < (localHeightMapWindow/2) || localHeightMapIndex(1) < (localHeightMapWindow/2))||(localHeightMapIndex(0) > (map_.getSize()(0)-localHeightMapWindow/2) || localHeightMapIndex(0) > (map_.getSize()(1)-localHeightMapWindow/2)))
        {
            return false;
        }
        else{
            gridMapPub(localHeightMapIndex);
            return true;
        }    
    }

    void CNNDataSet::gridMapPub(grid_map::Index mapCenterIndex)
    {
        //读取全局高度地图中机器人位置处的局部地图信息到二维vector容器中
        grid_map::Matrix& elevationData = map_["elevation"];
        float heightData,mapData;
        vector<vector<float>> orinElevationSubMap;
        vector<float> tmp;
        Index mapStartIndex(mapCenterIndex(0)-localHeightMapWindow, mapCenterIndex(1)-localHeightMapWindow);
        Index mapBufferSize(localHeightMapWindow, localHeightMapWindow);
        ros::Rate rate(30.0);
        ros::Time time = ros::Time::now();

        for (SubmapIterator it(map_, mapStartIndex, mapBufferSize); !it.isPastEnd(); ++it)
        {
            const grid_map::Index index(*it);
            heightData = elevationData(index(0), index(1));
            tmp.push_back(heightData);
        }

        for (int i = 0; i < localHeightMapWindow; i++)
        {
            vector<float> temp;
            int numRow = i * localHeightMapWindow;
            temp.assign(tmp.begin() + numRow, tmp.begin() + numRow + localHeightMapWindow);
            orinElevationSubMap.push_back(temp);
        }
        
        //构建grid map地图发布
        grid_map::GridMap map({"elevation"});
        map.setFrameId("map");
        map.setGeometry(Length(localHeightMapWindow * resolution_,localHeightMapWindow * resolution_),resolution_);
        for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
        {
            const grid_map::Index index2(*iterator);
            Position position;
            map.getPosition(*iterator, position);
            map.at("elevation", *iterator) = orinElevationSubMap[index2(0)][index2(1)];
        }
        map.setTimestamp(time.toNSec());
        grid_map_msgs::GridMap localMap;
        GridMapRosConverter::toMessage(map,localMap);
        localMapPublisher_.publish(localMap);
        rate.sleep();
        return;
    }

    void CNNDataSet::imuDataSetCallback(const sensor_msgs::Imu& imu_data)
    {
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

    // void DataSetForCNN::dataSetFlagCallback(const std_msgs::Bool& flag_msg)
    // {
    //     dataSetFlag = flag_msg.data;
    // }

// txt write to data
    void CNNDataSet::txtWrite(string name_)
    {
        // ros::Time Time_t = ros::Time::now();
        // double currentTime = Time_t.toSec();
        // time_t timep;
        // static char name[256] = {0};
        // time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep
        // strftime( name, sizeof(name), "%Y.%m.%d %H-%M-%S.txt",localtime(&timep) );
        // string timeName = name;
        // fileName = filePathName + fileName + timeName;//fileName is dir path
        // const char* pathName = fileName.data();
        // Mkdir(pathName);
        ofstream file;
        string txtName = "data";
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


    void CNNDataSet::gridMapCallback(const grid_map_msgs::GridMap& msg)
    {
        ROS_INFO("Saving map received from: %s to file %s.", nh.resolveName(gridMapTopic_).c_str(), fileName.c_str());
        grid_map::GridMap map;
        cv_bridge::CvImage image;
        grid_map::GridMapRosConverter::fromMessage(msg, map, {"elevation"});
        grid_map::GridMapRosConverter::toCvImage(map,"elevation", sensor_msgs::image_encodings::MONO8, image);
        bool success = cv::imwrite(fileName.c_str(),image.image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
        ROS_INFO("Success writing image: %s", success?"true":"false");
    }

    void CNNDataSet::robotPositionUpdate()
    {
        ROS_INFO("Reset robot position..........");
        objstate.request.model_state.model_name = "mobile_base";
        objstate.request.model_state.pose.position.x = randomNumber(0,20);
        objstate.request.model_state.pose.position.y = randomNumber(0,20);
        objstate.request.model_state.pose.position.z = randomNumber(0,5);
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

        robotCmdVelPub();
        // ROS_INFO("Robot velocity is published!");
        // double positionTemp_x = robotPosition_x, positionTemp_y = robotPosition_y;
        // ros::Rate r(0.1);
        // r.sleep();
        // ROS_INFO("Waitting robot state change .........");
        // //std::cout<<"robotPosition_x:"<<robotPosition_x<<"  "<<"robotPosition_y:"<<robotPosition_y<<"  "<<"positionTemp_x"<<positionTemp_x<<"  "<<"positionTemp_y"<<positionTemp_y<<std::endl;
        // ROS_ERROR("robotPosition_x:%.4f, robotPosition_y:%.4f, positionTemp_x:%.4f, positionTemp_y:%.4f",robotPosition_x,robotPosition_y,positionTemp_x,positionTemp_y);
        // if((robotPosition_x >= (positionTemp_x - 0.01) || robotPosition_x <= (positionTemp_x + 0.01)) && (robotPosition_y >= (positionTemp_y - 0.01) || robotPosition_y <= (positionTemp_y + 0.01)))
        //     {
        //     return false;
        //     }
        // else{   
        //     robotPositionBegin_x = robotPosition_x;
        //     robotPositionBegin_y = robotPosition_y;
        //     return true;
        // }
        
    }

    bool CNNDataSet::IsRobotStateNormal()
    {
        double positionTemp_x = robotPosition_x, positionTemp_y = robotPosition_y;
        ros::Rate r(0.1);
        r.sleep();
        ROS_INFO("Waitting robot state change .........");
        //std::cout<<"robotPosition_x:"<<robotPosition_x<<"  "<<"robotPosition_y:"<<robotPosition_y<<"  "<<"positionTemp_x"<<positionTemp_x<<"  "<<"positionTemp_y"<<positionTemp_y<<std::endl;
        ROS_ERROR("robotPosition_x:%.4f, robotPosition_y:%.4f, positionTemp_x:%.4f, positionTemp_y:%.4f",robotPosition_x,robotPosition_y,positionTemp_x,positionTemp_y);
        if((robotPosition_x >= (positionTemp_x - 0.01) || robotPosition_x <= (positionTemp_x + 0.01)) && (robotPosition_y >= (positionTemp_y - 0.01) || robotPosition_y <= (positionTemp_y + 0.01)))
            {
            return false;
            }
        else{   
            robotPositionBegin_x = robotPosition_x;
            robotPositionBegin_y = robotPosition_y;
            return true;
        }
    }

    float CNNDataSet::randomNumber(int rangeMin, int rangeMax)
    {
        float number;
        int precision = 100;//要使生成的随机数精确到小数点后几位，就让precision等于10的几次方
        int zuo = rangeMin * precision;//随机数的范围下限0
        int you = rangeMax * precision;//随机数的范围上限1
        float fenmu = 1.0 * precision;
        srand((unsigned int)time(NULL));
        number = (rand() % (you - zuo + 1) + zuo) / fenmu;

        return number;
    }

    void CNNDataSet::robotCmdVelPub()
    {

        cmdVel.linear.x = randomNumber(0,5);
        cmdVel.linear.y = randomNumber(0,5);
        cmdVel.linear.z = 0.0;

        cmdVel.angular.x = 0.0;
        cmdVel.angular.y = 0.0;
        cmdVel.angular.z = 0.0;

        robotLinearVelocity = sqrt(pow(cmdVel.linear.x, 2) + pow(cmdVel.linear.y, 2));
        cmd_velPublisher_.publish(cmdVel);

    }

    // 生成用户自定义范围的随机数
    double CNNDataSet::generateRandomNumber(double minRange, double maxRange) {
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


    bool CNNDataSet::robotRotation(double targetAngle)
    {
        cmdVel.linear.x = 0.0;
        cmdVel.linear.y = 0.0;
        cmdVel.linear.z = 0.0;

        cmdVel.angular.x = 0.0;
        cmdVel.angular.y = 0.0;
        cmdVel.angular.z = 1.0;

        cmd_velPublisher_.publish(cmdVel);

        if(robotAngleOfYaw <= (targetAngle + 0.01) || robotAngleOfYaw <= (targetAngle - 0.01) )
        {    
            cmdVel.linear.x = 0.0;
            cmdVel.linear.y = 0.0;
            cmdVel.linear.z = 0.0;

            cmdVel.angular.x = 0.0;
            cmdVel.angular.y = 0.0;
            cmdVel.angular.z = 0.0;
            return true;
        }
        else{
            return false;
        }
            
    }

    void CNNDataSet::Mkdir(const char* pathName)
    {
        
        int isCreate = mkdir(pathName,S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        if( !isCreate )
        ROS_INFO("create path:%s",pathName);
        else
        ROS_ERROR("create path failed! error code : %s",isCreate,pathName);
    }


    void CNNDataSet::dataSetStart()
    {
        // if(robotPositionUpdate())
        // {
        //     // ros::Time Time_t = ros::Time::now();
        //     // double currentTime = Time_t.toSec();
        //     // time_t timep;
        //     // static char name[256] = {0};
        //     // time(&timep);//获取从1970至今过了多少秒，存入time_t类型的timep
        //     // strftime( name, sizeof(name), "%Y.%m.%d %H-%M-%S.txt",localtime(&timep) );
        //     // string timeName = name;
        //     // fileName = filePathName + fileName + timeName;//fileName is dir path
        //     // const char* pathName = fileName.data();
        //     // Mkdir(pathName);


        //     if(robotStateDataSet())
        //     {
        //         txtWrite(fileName);
        //         ROS_INFO("All function is OK!");
        //         return;
        //     }
        //     else{
        //         ROS_ERROR("robotStateDataSet() is false!");
        //     }
        // }else{
        //      ROS_ERROR("robotPositionUpdate()is false!");
        // }
        // do 
        //     robotPositionUpdate();
        // while(!IsRobotStateNormal())
        // {
        //     robotPositionUpdate();
        // }

        if(!IsRobotStateNormal())
        {
            robotPositionUpdate();
        }else{
            if(robotRotation(generateRandomNumber(-1.57,1.57)))
            {
                robotCmdVelPub();
                ROS_INFO("Robot velocity is published!");
                robotStateDataSet();
                txtWrite(fileName);
                return;
            }
            else{
                return;
            }
        }

    }

int main(int argc, char **argv)
{
    ros::init(argc,argv,"data_set");
    CNNDataSet DataSet;
    while (ros::ok())
    {
        DataSet.dataSetStart();
        ROS_INFO("One times data set is finished");
    }
    ros::spin();
    return 0;
}



